"""ROS 2 physical boundary for a simulator-neutral environment backend."""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time
from typing import Any, Callable, Mapping, TypeVar

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

from .backend_protocol import EnvironmentBackend, EpisodeMetrics, Observation
from .http_backend import HttpEnvironmentBackend
from .ros_conversion import (
    decision_payload,
    goal_text_message,
    image_message,
    odometry_message,
    pose_message,
    run_context,
)


_Result = TypeVar("_Result")


@dataclass
class _Episode:
    generation: int
    run_id: str
    episode_id: str
    observation: Observation
    context: RunContext
    deadline: float
    metrics: EpisodeMetrics | None = None
    operation_in_flight: bool = False
    terminal: bool = False
    terminal_published: bool = False
    service_released: bool = False
    error: str = ""


class EnvNode(Node):
    """Expose one backend episode at a time through canonical LuxiNav endpoints."""

    _IDLE = "IDLE"
    _RESETTING = "RESETTING"
    _ACTIVE = "ACTIVE"
    _DRAINING = "DRAINING"
    _SHUTTING_DOWN = "SHUTTING_DOWN"
    _SHUTDOWN = "SHUTDOWN"

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
        self._backend_lock = threading.Lock()
        self._shutdown_once_lock = threading.Lock()
        self._phase = self._IDLE
        self._generation = 0
        self._episode: _Episode | None = None
        self._last_stamp_ns = -1
        self._backend_shutdown_done = False
        self._backend_shutdown_result: Mapping[str, Any] = {
            "status": "shutting_down"
        }

    @property
    def shutdown_requested(self) -> bool:
        with self._condition:
            return self._phase in {self._SHUTTING_DOWN, self._SHUTDOWN}

    @staticmethod
    def _remaining(deadline: float) -> float:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            raise TimeoutError("evaluation deadline exceeded")
        return remaining

    def _backend_call(
        self,
        operation: Callable[..., _Result],
        deadline: float,
        *args,
        **kwargs,
    ) -> _Result:
        remaining = self._remaining(deadline)
        if not self._backend_lock.acquire(timeout=remaining):
            raise TimeoutError("evaluation deadline exceeded waiting for backend")
        try:
            kwargs["timeout_seconds"] = self._remaining(deadline)
            try:
                return operation(*args, **kwargs)
            except TimeoutError as error:
                raise TimeoutError("evaluation deadline exceeded") from error
        finally:
            self._backend_lock.release()

    def _next_stamp_locked(self) -> Time:
        now_ns = self.get_clock().now().nanoseconds
        stamp_ns = max(now_ns, self._last_stamp_ns + 1)
        self._last_stamp_ns = stamp_ns
        return Time(
            sec=stamp_ns // 1_000_000_000,
            nanosec=stamp_ns % 1_000_000_000,
        )

    def _context_for_locked(self, observation: Observation) -> RunContext:
        return run_context(
            observation,
            self._next_stamp_locked(),
            contract_version=self._contract_version,
            source_plugin_id=self._source_plugin_id,
        )

    def _publish_observation_locked(
        self, observation: Observation, context: RunContext
    ) -> None:
        stamp = context.stamp
        self._context_pub.publish(context)
        self._rgb_pub.publish(image_message(observation.rgb, stamp, "camera"))
        self._depth_pub.publish(image_message(observation.depth, stamp, "camera"))
        self._pose_pub.publish(pose_message(observation, stamp))
        self._odometry_pub.publish(odometry_message(observation, stamp))
        self._goal_pub.publish(goal_text_message(observation))

    def _publish_state_locked(
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

    def _terminalize_locked(self, episode: _Episode, error: str = "") -> None:
        if episode.terminal:
            return
        episode.terminal = True
        episode.error = error
        if not episode.terminal_published:
            steps = 0 if episode.metrics is None else episode.metrics.steps
            self._publish_state_locked(
                episode.context,
                EpisodeState.EPISODE_FINISHED,
                steps,
                error,
            )
            episode.terminal_published = True
        self._condition.notify_all()

    def _release_episode_locked(self, episode: _Episode) -> None:
        if self._episode is not episode:
            return
        self._episode = None
        if self._phase not in {self._SHUTTING_DOWN, self._SHUTDOWN}:
            self._phase = self._IDLE
        self._condition.notify_all()

    def _finish_drain_locked(self, episode: _Episode) -> None:
        episode.operation_in_flight = False
        if episode.service_released:
            self._release_episode_locked(episode)
        else:
            self._condition.notify_all()

    def _on_readiness(self, request, response):
        del request
        response.component_id = self._source_plugin_id
        with self._condition:
            if self._phase in {self._SHUTTING_DOWN, self._SHUTDOWN}:
                response.ready = False
                response.status = "shutting_down"
                response.error = "environment is shutting down"
                return response
        try:
            deadline = time.monotonic() + self._evaluation_timeout_seconds
            health = self._backend_call(self._backend.health, deadline)
            response.status = str(health.get("status", "unknown"))
            response.ready = response.status == "ok"
            if not response.ready:
                response.error = f"backend status is {response.status}"
        except Exception as error:
            response.ready = False
            response.status = "unavailable"
            response.error = str(error)
        return response

    def _reserve_evaluation(self, response) -> int | None:
        with self._condition:
            if self._phase in {self._SHUTTING_DOWN, self._SHUTDOWN}:
                response.accepted = False
                response.error = "environment is shutting down"
                return None
            if self._phase != self._IDLE:
                response.accepted = False
                response.error = "an evaluation is already active"
                return None
            self._generation += 1
            self._phase = self._RESETTING
            return self._generation

    def _on_evaluate_episode(self, request, response):
        deadline = time.monotonic() + self._evaluation_timeout_seconds
        generation = self._reserve_evaluation(response)
        if generation is None:
            return response
        if not request.run_id or not request.episode_selector or request.max_steps < 1:
            with self._condition:
                if self._phase == self._RESETTING:
                    self._phase = self._IDLE
            response.accepted = False
            response.error = (
                "run_id, episode_selector, and a positive max_steps are required"
            )
            return response

        episode: _Episode | None = None
        metrics: EpisodeMetrics | None = None
        error = ""
        try:
            observation = self._backend_call(
                self._backend.reset,
                deadline,
                request.run_id,
                request.episode_selector,
                seed=0,
                max_steps=request.max_steps,
            )
            with self._condition:
                if (
                    self._generation != generation or
                    self._phase in {self._SHUTTING_DOWN, self._SHUTDOWN}
                ):
                    raise RuntimeError("environment shutdown interrupted reset")
                context = self._context_for_locked(observation)
                episode = _Episode(
                    generation=generation,
                    run_id=observation.run_id,
                    episode_id=observation.episode_id,
                    observation=observation,
                    context=context,
                    deadline=deadline,
                )
                self._episode = episode
                self._phase = self._ACTIVE
                response.accepted = True
                self._publish_observation_locked(observation, context)
                if observation.state == "EPISODE_FINISHED":
                    self._terminalize_locked(episode)
                else:
                    self._publish_state_locked(
                        context, EpisodeState.READY, 0
                    )

                while not episode.terminal:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0.0:
                        self._terminalize_locked(
                            episode, "evaluation deadline exceeded"
                        )
                        if episode.operation_in_flight:
                            self._phase = self._DRAINING
                        break
                    self._condition.wait(timeout=remaining)
                metrics = episode.metrics
                error = episode.error

            if not error:
                try:
                    metrics = self._backend_call(
                        self._backend.metrics, deadline
                    )
                except Exception as metrics_error:
                    error = f"final metrics unavailable: {metrics_error}"
        except Exception as exception:
            error = str(exception)
            if episode is None:
                response.accepted = False
        finally:
            with self._condition:
                if episode is None:
                    if self._phase == self._RESETTING:
                        self._phase = self._IDLE
                elif episode.operation_in_flight:
                    episode.service_released = True
                    if self._phase not in {
                        self._SHUTTING_DOWN,
                        self._SHUTDOWN,
                    }:
                        self._phase = self._DRAINING
                else:
                    self._release_episode_locked(episode)

        response.episode_id = (
            request.episode_selector
            if episode is None
            else episode.observation.episode_id
        )
        if metrics is not None:
            response.scene_id = metrics.scene_id
            response.success = metrics.success
            response.spl = metrics.spl
            response.distance_to_goal = metrics.distance_to_goal
            response.steps = metrics.steps
            response.simulator_seconds = metrics.simulator_seconds
        response.error = error
        return response

    def _on_decision(self, message: Decision) -> None:
        with self._condition:
            episode = self._episode
            if (
                self._phase != self._ACTIVE or
                episode is None or
                episode.terminal or
                episode.operation_in_flight or
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
            episode.operation_in_flight = True
            steps = 0 if episode.metrics is None else episode.metrics.steps
            self._publish_state_locked(
                episode.context, EpisodeState.RUNNING, steps
            )

        try:
            result = self._backend_call(
                self._backend.step,
                episode.deadline,
                payload,
            )
        except Exception as error:
            with self._condition:
                if self._episode is episode:
                    if not episode.terminal:
                        self._terminalize_locked(episode, str(error))
                    self._finish_drain_locked(episode)
            return

        with self._condition:
            if self._episode is not episode:
                return
            if episode.terminal or self._phase != self._ACTIVE:
                self._finish_drain_locked(episode)
                return

            next_context = self._context_for_locked(result.observation)
            episode.observation = result.observation
            episode.context = next_context
            episode.metrics = result.metrics
            episode.operation_in_flight = False
            terminal = (
                result.observation.state == "EPISODE_FINISHED" or
                result.metrics.success or
                payload.kind == "stop"
            )
            self._publish_observation_locked(result.observation, next_context)
            self._publish_state_locked(
                next_context,
                EpisodeState.ACTION_FINISHED,
                result.metrics.steps,
            )
            if terminal:
                self._terminalize_locked(episode)

    def _begin_shutdown(self, reason: str) -> None:
        with self._condition:
            if self._phase == self._SHUTDOWN:
                return
            self._phase = self._SHUTTING_DOWN
            if self._episode is not None:
                self._terminalize_locked(self._episode, reason)
            self._condition.notify_all()

    def _shutdown_backend(self, deadline: float) -> Mapping[str, Any]:
        with self._shutdown_once_lock:
            if self._backend_shutdown_done:
                return self._backend_shutdown_result
            result = self._backend_call(
                self._backend.shutdown,
                deadline,
            )
            with self._condition:
                self._backend_shutdown_result = result
                self._backend_shutdown_done = True
                self._phase = self._SHUTDOWN
                self._condition.notify_all()
            return result

    def close(self) -> None:
        """Terminalize active work and shut down the backend exactly once."""
        self._begin_shutdown("process shutdown")
        try:
            self._shutdown_backend(
                time.monotonic() + self._evaluation_timeout_seconds
            )
        except Exception as error:
            self.get_logger().error(f"backend shutdown failed: {error}")

    def _on_shutdown(self, request, response):
        del request
        self._begin_shutdown("shutdown requested")
        try:
            result = self._shutdown_backend(
                time.monotonic() + self._evaluation_timeout_seconds
            )
            response.success = True
            response.message = str(result.get("status", "shutting_down"))
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
        node.close()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
