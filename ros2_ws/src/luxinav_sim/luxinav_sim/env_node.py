"""ROS 2 physical boundary for a simulator-neutral environment backend."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import threading
import time
from typing import Any, Mapping

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from luxinav_interfaces.msg import Decision, EpisodeState, RunContext
from luxinav_interfaces.srv import EvaluateEpisode, Readiness
from nav_msgs.msg import Odometry
import rclpy
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .backend_protocol import EnvironmentBackend, Observation
from .http_backend import HttpEnvironmentBackend
from .ros_conversion import (
    decision_payload,
    goal_text_message,
    image_message,
    odometry_message,
    pose_message,
    run_context,
)


@dataclass
class _Episode:
    run_id: str
    episode_id: str
    observation: Observation
    context: RunContext
    metrics: dict[str, Any] = field(
        default_factory=lambda: {"steps": 0, "success": False}
    )
    decision_in_flight: bool = False
    terminal: bool = False
    error: str = ""


class EnvNode(Node):
    """Expose one backend episode at a time through canonical LuxiNav endpoints."""

    def __init__(
        self,
        backend: EnvironmentBackend | None = None,
        *,
        source_plugin_id: str = "mock",
        contract_version: str = "luxinav.v1",
        evaluation_timeout_seconds: float = 30.0,
        context=None,
    ) -> None:
        super().__init__("luxinav_env", context=context)
        self.declare_parameter("backend_url", "http://127.0.0.1:18090")
        self.declare_parameter("source_plugin_id", source_plugin_id)
        self.declare_parameter(
            "evaluation_timeout_seconds", evaluation_timeout_seconds
        )
        self._source_plugin_id = (
            self.get_parameter("source_plugin_id").get_parameter_value().string_value
        )
        self._contract_version = contract_version
        self._evaluation_timeout_seconds = (
            self.get_parameter("evaluation_timeout_seconds")
            .get_parameter_value()
            .double_value
        )
        if self._evaluation_timeout_seconds <= 0.0:
            raise ValueError("evaluation_timeout_seconds must be positive")
        if backend is None:
            backend_url = (
                self.get_parameter("backend_url").get_parameter_value().string_value
            )
            backend = HttpEnvironmentBackend(backend_url)
        self._backend: EnvironmentBackend = backend

        reliable = QoSProfile(
            depth=10, reliability=QoSReliabilityPolicy.RELIABLE
        )
        self._context_pub = self.create_publisher(
            RunContext, "/luxinav/observation/context", reliable
        )
        self._rgb_pub = self.create_publisher(
            Image, "/luxinav/observation/rgb", qos_profile_sensor_data
        )
        self._depth_pub = self.create_publisher(
            Image, "/luxinav/observation/depth", qos_profile_sensor_data
        )
        self._pose_pub = self.create_publisher(
            PoseStamped, "/luxinav/observation/pose", reliable
        )
        self._odometry_pub = self.create_publisher(
            Odometry, "/luxinav/observation/odometry", qos_profile_sensor_data
        )
        self._goal_pub = self.create_publisher(
            String, "/luxinav/goal/text", reliable
        )
        self._state_pub = self.create_publisher(
            EpisodeState, "/luxinav/episode/state", reliable
        )

        self._decision_group = MutuallyExclusiveCallbackGroup()
        self._service_group = ReentrantCallbackGroup()
        self._decision_sub = self.create_subscription(
            Decision,
            "/luxinav/decision",
            self._on_decision,
            reliable,
            callback_group=self._decision_group,
        )
        self._readiness_service = self.create_service(
            Readiness,
            "/luxinav/env/readiness",
            self._on_readiness,
            callback_group=self._service_group,
        )
        self._evaluate_service = self.create_service(
            EvaluateEpisode,
            "/luxinav/env/evaluate_episode",
            self._on_evaluate_episode,
            callback_group=self._service_group,
        )
        self._shutdown_service = self.create_service(
            Trigger,
            "/luxinav/env/shutdown",
            self._on_shutdown,
            callback_group=self._service_group,
        )

        self._condition = threading.Condition()
        self._evaluation_active = False
        self._episode: _Episode | None = None
        self._last_stamp_ns = -1
        self._shutdown_requested = False

    @property
    def shutdown_requested(self) -> bool:
        return self._shutdown_requested

    def _next_stamp(self) -> Time:
        now_ns = self.get_clock().now().nanoseconds
        with self._condition:
            stamp_ns = max(now_ns, self._last_stamp_ns + 1)
            self._last_stamp_ns = stamp_ns
        return Time(
            sec=stamp_ns // 1_000_000_000,
            nanosec=stamp_ns % 1_000_000_000,
        )

    def _context_for(self, observation: Observation) -> RunContext:
        return run_context(
            observation,
            self._next_stamp(),
            contract_version=self._contract_version,
            source_plugin_id=self._source_plugin_id,
        )

    def _publish_observation(
        self, observation: Observation, context: RunContext
    ) -> None:
        stamp = context.stamp
        self._context_pub.publish(context)
        self._rgb_pub.publish(image_message(observation.rgb, stamp, "camera"))
        self._depth_pub.publish(image_message(observation.depth, stamp, "camera"))
        self._pose_pub.publish(pose_message(observation, stamp))
        self._odometry_pub.publish(odometry_message(observation, stamp))
        self._goal_pub.publish(goal_text_message(observation))

    def _publish_state(
        self, context: RunContext, state: int, steps: int, detail: str = ""
    ) -> None:
        self._state_pub.publish(
            EpisodeState(
                context=context,
                state=state,
                step_count=steps,
                detail=detail,
            )
        )

    def _on_readiness(self, request, response):
        del request
        response.component_id = self._source_plugin_id
        try:
            health = self._backend.health()
            response.status = str(health.get("status", "unknown"))
            response.ready = response.status == "ok"
            if not response.ready:
                response.error = f"backend status is {response.status}"
        except Exception as error:
            response.ready = False
            response.status = "unavailable"
            response.error = str(error)
        return response

    def _on_evaluate_episode(self, request, response):
        with self._condition:
            if self._evaluation_active:
                response.accepted = False
                response.error = "an evaluation is already active"
                return response
            self._evaluation_active = True

        if not request.run_id or not request.episode_selector or request.max_steps < 1:
            with self._condition:
                self._evaluation_active = False
                self._condition.notify_all()
            response.accepted = False
            response.error = (
                "run_id, episode_selector, and a positive max_steps are required"
            )
            return response

        started = time.monotonic()
        episode_started = False
        try:
            observation = self._backend.reset(
                request.run_id,
                request.episode_selector,
                seed=0,
                max_steps=request.max_steps,
            )
            context = self._context_for(observation)
            episode = _Episode(
                run_id=observation.run_id,
                episode_id=observation.episode_id,
                observation=observation,
                context=context,
            )
            episode_started = True
            response.accepted = True
            with self._condition:
                self._episode = episode
            self._publish_observation(observation, context)
            self._publish_state(context, EpisodeState.READY, 0)

            deadline = started + self._evaluation_timeout_seconds
            with self._condition:
                while not episode.terminal:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0.0:
                        episode.terminal = True
                        episode.error = "evaluation deadline exceeded"
                        self._publish_state(
                            episode.context,
                            EpisodeState.EPISODE_FINISHED,
                            int(episode.metrics.get("steps", 0)),
                            episode.error,
                        )
                        break
                    self._condition.wait(timeout=remaining)
                metrics = dict(episode.metrics)
                final_observation = episode.observation
                error = episode.error
            try:
                metrics.update(dict(self._backend.metrics()))
            except Exception as metrics_error:
                if not error:
                    error = f"final metrics unavailable: {metrics_error}"
        except Exception as exception:
            metrics = {"steps": 0, "success": False}
            final_observation = None
            error = str(exception)
            if not episode_started:
                response.accepted = False
        finally:
            with self._condition:
                self._episode = None
                self._evaluation_active = False
                self._condition.notify_all()

        response.episode_id = (
            request.episode_selector
            if final_observation is None
            else final_observation.episode_id
        )
        response.scene_id = self._source_plugin_id
        response.success = bool(metrics.get("success", False))
        response.spl = float(metrics.get("spl", 1.0 if response.success else 0.0))
        response.steps = int(metrics.get("steps", 0))
        response.simulator_seconds = float(metrics.get("simulator_seconds", 0.0))
        if "distance_to_goal" in metrics:
            response.distance_to_goal = float(metrics["distance_to_goal"])
        elif final_observation is not None:
            response.distance_to_goal = math.hypot(
                float(final_observation.goal["x"]) -
                float(final_observation.pose["x"]),
                float(final_observation.goal["y"]) -
                float(final_observation.pose["y"]),
            )
        response.error = error
        return response

    def _on_decision(self, message: Decision) -> None:
        with self._condition:
            episode = self._episode
            if (
                episode is None or
                episode.terminal or
                episode.decision_in_flight or
                message.context.run_id != episode.run_id or
                message.context.episode_id != episode.episode_id or
                message.context.frame_id != episode.context.frame_id
            ):
                return
            try:
                payload = decision_payload(message)
            except ValueError as error:
                self.get_logger().warning(str(error))
                return
            episode.decision_in_flight = True
            current_context = episode.context
            current_steps = int(episode.metrics.get("steps", 0))

        self._publish_state(
            current_context, EpisodeState.RUNNING, current_steps
        )
        try:
            result = self._backend.step(payload)
            next_context = self._context_for(result.observation)
        except Exception as error:
            with self._condition:
                if self._episode is episode:
                    episode.decision_in_flight = False
                    episode.terminal = True
                    episode.error = str(error)
                    self._publish_state(
                        episode.context,
                        EpisodeState.FAILED,
                        current_steps,
                        episode.error,
                    )
                    self._condition.notify_all()
            return

        with self._condition:
            if self._episode is not episode or episode.terminal:
                return
            episode.observation = result.observation
            episode.context = next_context
            episode.metrics = dict(result.metrics)
            episode.decision_in_flight = False
            steps = int(episode.metrics.get("steps", 0))
            terminal = (
                result.observation.state == "EPISODE_FINISHED" or
                bool(episode.metrics.get("success", False)) or
                payload.kind == "stop"
            )

        self._publish_observation(result.observation, next_context)
        self._publish_state(next_context, EpisodeState.ACTION_FINISHED, steps)
        if terminal:
            self._publish_state(
                next_context, EpisodeState.EPISODE_FINISHED, steps
            )
            with self._condition:
                if self._episode is episode:
                    episode.terminal = True
                    self._condition.notify_all()

    def _on_shutdown(self, request, response):
        del request
        try:
            result: Mapping[str, Any] = self._backend.shutdown()
            response.success = True
            response.message = str(result.get("status", "shutting_down"))
            self._shutdown_requested = True
        except Exception as error:
            response.success = False
            response.message = str(error)
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EnvNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        while rclpy.ok() and not node.shutdown_requested:
            executor.spin_once(timeout_sec=0.1)
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
