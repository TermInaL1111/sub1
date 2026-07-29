import math
import threading
import time

from geometry_msgs.msg import PointStamped, PoseStamped
from luxinav_interfaces.msg import Decision, EpisodeState, RunContext
from luxinav_interfaces.srv import EvaluateEpisode, Readiness
from luxinav_sim.backend_protocol import (
    EpisodeMetrics,
    ImagePayload,
    Observation,
    StepResult,
)
from luxinav_sim.env_node import EnvNode
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    qos_profile_sensor_data,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger


def _observation(run_id, episode_id, frame_id, *, terminal=False):
    return Observation(
        run_id=run_id,
        episode_id=episode_id,
        frame_id=frame_id,
        rgb=ImagePayload(2, 1, "rgb8", bytes(range(6))),
        depth=ImagePayload(2, 1, "32FC1", b"\x00\x00\x80?\x00\x00\x00@"),
        pose={"x": float(frame_id), "y": 0.25, "yaw": math.pi / 2},
        goal={"text": "mock target", "x": 0.5, "y": 0.0},
        state="EPISODE_FINISHED" if terminal else "READY",
    )


class RecordingBackend:
    def __init__(self):
        self.reset_calls = []
        self.step_calls = []
        self.shutdown_calls = 0
        self._run_id = ""
        self._episode_id = ""
        self._max_steps = 0
        self._steps = 0
        self._success = False
        self.timeouts = []

    def health(self, *, timeout_seconds=None):
        self.timeouts.append(("health", timeout_seconds))
        return {"status": "ok"}

    def reset(
        self,
        run_id,
        episode_id,
        seed,
        max_steps,
        *,
        timeout_seconds=None,
    ):
        self.timeouts.append(("reset", timeout_seconds))
        self.reset_calls.append((run_id, episode_id, seed, max_steps))
        self._run_id = run_id
        self._episode_id = episode_id
        self._max_steps = max_steps
        self._steps = 0
        self._success = False
        return _observation(run_id, episode_id, 0)

    def step(self, decision, *, timeout_seconds=None):
        self.timeouts.append(("step", timeout_seconds))
        self.step_calls.append(decision)
        self._steps += 1
        terminal = decision.kind == "stop" or self._steps >= self._max_steps
        self._success = decision.kind == "stop"
        return StepResult(
            _observation(
                self._run_id,
                self._episode_id,
                self._steps,
                terminal=terminal,
            ),
            self._metrics(),
        )

    def metrics(self, *, timeout_seconds=None):
        self.timeouts.append(("metrics", timeout_seconds))
        return self._metrics()

    def shutdown(self, *, timeout_seconds=None):
        self.timeouts.append(("shutdown", timeout_seconds))
        self.shutdown_calls += 1
        return {"status": "shutting_down"}

    def _metrics(self):
        return EpisodeMetrics(
            scene_id="mock-scene",
            success=self._success,
            spl=0.25,
            distance_to_goal=0.125,
            steps=self._steps,
            simulator_seconds=1.5,
        )


class BlockingBackend(RecordingBackend):
    def __init__(self, *, block_operation):
        super().__init__()
        self.block_operation = block_operation
        self.operation_started = threading.Event()
        self.operation_finished = threading.Event()
        self.release_operation = threading.Event()
        self._active_lock = threading.Lock()
        self._active_calls = 0
        self.max_active_calls = 0

    def _enter(self):
        with self._active_lock:
            self._active_calls += 1
            self.max_active_calls = max(self.max_active_calls, self._active_calls)

    def _leave(self):
        with self._active_lock:
            self._active_calls -= 1

    def _block(self, timeout_seconds):
        self.operation_started.set()
        completed = self.release_operation.wait(timeout_seconds or 2.0)
        self.operation_finished.set()
        if not completed:
            raise TimeoutError(f"{self.block_operation} timed out")

    def reset(
        self,
        run_id,
        episode_id,
        seed,
        max_steps,
        *,
        timeout_seconds=None,
    ):
        self._enter()
        try:
            if self.block_operation == "reset":
                self.timeouts.append(("reset", timeout_seconds))
                self._block(timeout_seconds)
            return super().reset(
                run_id,
                episode_id,
                seed,
                max_steps,
                timeout_seconds=timeout_seconds,
            )
        finally:
            self._leave()

    def step(self, decision, *, timeout_seconds=None):
        self._enter()
        try:
            if self.block_operation == "step":
                self.timeouts.append(("step", timeout_seconds))
                self._block(timeout_seconds)
            return super().step(decision, timeout_seconds=timeout_seconds)
        finally:
            self._leave()

    def metrics(self, *, timeout_seconds=None):
        self._enter()
        try:
            if self.block_operation == "metrics":
                self.timeouts.append(("metrics", timeout_seconds))
                self._block(timeout_seconds)
            return super().metrics(timeout_seconds=timeout_seconds)
        finally:
            self._leave()

    def shutdown(self, *, timeout_seconds=None):
        self._enter()
        try:
            return super().shutdown(timeout_seconds=timeout_seconds)
        finally:
            self._leave()


class EnvHarness:
    def __init__(self, evaluation_timeout_seconds=1.0, backend=None):
        self.backend = backend or RecordingBackend()
        self.context = Context()
        rclpy.init(context=self.context)
        self.env = EnvNode(
            self.backend,
            context=self.context,
            source_plugin_id="mock",
            evaluation_timeout_seconds=evaluation_timeout_seconds,
        )
        self.probe = Node("env_contract_probe", context=self.context)
        self.messages = {
            "context": [],
            "rgb": [],
            "depth": [],
            "pose": [],
            "odometry": [],
            "goal": [],
            "goal_vector": [],
            "state": [],
        }
        reliable = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        self.subscriptions = [
            self.probe.create_subscription(
                RunContext,
                "/luxinav/observation/context",
                lambda msg: self.messages["context"].append(msg),
                reliable,
            ),
            self.probe.create_subscription(
                Image,
                "/luxinav/observation/rgb",
                lambda msg: self.messages["rgb"].append(msg),
                qos_profile_sensor_data,
            ),
            self.probe.create_subscription(
                Image,
                "/luxinav/observation/depth",
                lambda msg: self.messages["depth"].append(msg),
                qos_profile_sensor_data,
            ),
            self.probe.create_subscription(
                PoseStamped,
                "/luxinav/observation/pose",
                lambda msg: self.messages["pose"].append(msg),
                reliable,
            ),
            self.probe.create_subscription(
                Odometry,
                "/luxinav/observation/odometry",
                lambda msg: self.messages["odometry"].append(msg),
                qos_profile_sensor_data,
            ),
            self.probe.create_subscription(
                String,
                "/luxinav/goal/text",
                lambda msg: self.messages["goal"].append(msg),
                reliable,
            ),
            self.probe.create_subscription(
                PointStamped,
                "/luxinav/goal/vector",
                lambda msg: self.messages["goal_vector"].append(msg),
                reliable,
            ),
            self.probe.create_subscription(
                EpisodeState,
                "/luxinav/episode/state",
                lambda msg: self.messages["state"].append(msg),
                reliable,
            ),
        ]
        self.decision_pub = self.probe.create_publisher(
            Decision, "/luxinav/decision", reliable
        )
        self.evaluate_client = self.probe.create_client(
            EvaluateEpisode, "/luxinav/env/evaluate_episode"
        )
        self.readiness_client = self.probe.create_client(
            Readiness, "/luxinav/env/readiness"
        )
        self.shutdown_client = self.probe.create_client(
            Trigger, "/luxinav/env/shutdown"
        )
        self.executor = MultiThreadedExecutor(num_threads=4, context=self.context)
        self.executor.add_node(self.env)
        self.executor.add_node(self.probe)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        assert self.evaluate_client.wait_for_service(timeout_sec=2.0)
        assert self.readiness_client.wait_for_service(timeout_sec=2.0)
        assert self.shutdown_client.wait_for_service(timeout_sec=2.0)

    def close(self):
        self.executor.shutdown(timeout_sec=2.0)
        self.thread.join(timeout=2.0)
        self.probe.destroy_node()
        self.env.destroy_node()
        rclpy.shutdown(context=self.context)
        assert not self.thread.is_alive()

    def wait_for(self, predicate, timeout=2.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.01)
        raise AssertionError("condition was not observed before the deadline")

    def start_episode(self, run_id="r1", episode_id="mock-000", max_steps=3):
        request = EvaluateEpisode.Request(
            run_id=run_id,
            episode_selector=episode_id,
            max_steps=max_steps,
        )
        future = self.evaluate_client.call_async(request)
        self.wait_for(
            lambda: any(
                state.state == EpisodeState.READY and
                state.context.run_id == run_id and
                state.context.episode_id == episode_id
                for state in self.messages["state"]
            )
        )
        return future

    def publish_decision(
        self,
        run_id="r1",
        episode_id="mock-000",
        frame_id=0,
        kind=Decision.CONTINUOUS_CONTROL,
    ):
        decision = Decision()
        decision.context.contract_version = "luxinav.v1"
        decision.context.run_id = run_id
        decision.context.episode_id = episode_id
        decision.context.frame_id = frame_id
        decision.context.source_plugin_id = "agent"
        decision.kind = kind
        self.decision_pub.publish(decision)

    def wait_future(self, future, timeout=2.0):
        self.wait_for(future.done, timeout)
        return future.result()

    def topic_map(self):
        return dict(self.probe.get_topic_names_and_types())

    def service_map(self):
        return dict(self.probe.get_service_names_and_types())


@pytest.fixture
def env_harness():
    harness = EnvHarness()
    try:
        yield harness
    finally:
        harness.close()


def _stamp_tuple(stamp):
    return stamp.sec, stamp.nanosec


def test_env_node_exposes_canonical_graph_types_and_qos(env_harness):
    topics = env_harness.topic_map()
    services = env_harness.service_map()
    task_topics = sorted(name for name in topics if name.startswith("/luxinav/"))
    task_services = sorted(name for name in services if name.startswith("/luxinav/"))
    print(f"discovered LuxiNav topics: {task_topics}")
    print(f"discovered LuxiNav services: {task_services}")

    assert topics["/luxinav/observation/context"] == [
        "luxinav_interfaces/msg/RunContext"
    ]
    assert topics["/luxinav/observation/rgb"] == ["sensor_msgs/msg/Image"]
    assert topics["/luxinav/observation/depth"] == ["sensor_msgs/msg/Image"]
    assert topics["/luxinav/observation/pose"] == ["geometry_msgs/msg/PoseStamped"]
    assert topics["/luxinav/observation/odometry"] == ["nav_msgs/msg/Odometry"]
    assert topics["/luxinav/goal/text"] == ["std_msgs/msg/String"]
    assert topics["/luxinav/goal/vector"] == [
        "geometry_msgs/msg/PointStamped"
    ]
    assert topics["/luxinav/episode/state"] == [
        "luxinav_interfaces/msg/EpisodeState"
    ]
    assert topics["/luxinav/decision"] == ["luxinav_interfaces/msg/Decision"]
    assert services["/luxinav/env/readiness"] == [
        "luxinav_interfaces/srv/Readiness"
    ]
    assert services["/luxinav/env/evaluate_episode"] == [
        "luxinav_interfaces/srv/EvaluateEpisode"
    ]
    assert services["/luxinav/env/shutdown"] == ["std_srvs/srv/Trigger"]
    assert not any(name.startswith("/habitat/") for name in (*topics, *services))

    context_qos = env_harness.env.get_publishers_info_by_topic(
        "/luxinav/observation/context"
    )[0].qos_profile
    state_qos = env_harness.env.get_publishers_info_by_topic(
        "/luxinav/episode/state"
    )[0].qos_profile
    decision_qos = env_harness.env.get_subscriptions_info_by_topic(
        "/luxinav/decision"
    )[0].qos_profile
    rgb_qos = env_harness.env.get_publishers_info_by_topic(
        "/luxinav/observation/rgb"
    )[0].qos_profile
    depth_qos = env_harness.env.get_publishers_info_by_topic(
        "/luxinav/observation/depth"
    )[0].qos_profile
    odom_qos = env_harness.env.get_publishers_info_by_topic(
        "/luxinav/observation/odometry"
    )[0].qos_profile
    goal_vector_qos = env_harness.env.get_publishers_info_by_topic(
        "/luxinav/goal/vector"
    )[0].qos_profile
    assert (context_qos.reliability, context_qos.depth) == (
        QoSReliabilityPolicy.RELIABLE,
        10,
    )
    assert (state_qos.reliability, state_qos.depth) == (
        QoSReliabilityPolicy.RELIABLE,
        10,
    )
    assert (decision_qos.reliability, decision_qos.depth) == (
        QoSReliabilityPolicy.RELIABLE,
        10,
    )
    assert rgb_qos.reliability == QoSReliabilityPolicy.BEST_EFFORT
    assert (depth_qos.reliability, depth_qos.depth) == (
        qos_profile_sensor_data.reliability,
        qos_profile_sensor_data.depth,
    )
    assert odom_qos.reliability == QoSReliabilityPolicy.BEST_EFFORT
    assert (goal_vector_qos.reliability, goal_vector_qos.depth) == (
        QoSReliabilityPolicy.RELIABLE,
        10,
    )


def test_frame_identity_rejects_stale_foreign_and_duplicate_decisions(env_harness):
    evaluate = env_harness.start_episode(max_steps=1)
    env_harness.publish_decision(frame_id=99)
    env_harness.publish_decision(run_id="foreign-run")
    env_harness.publish_decision(episode_id="foreign-episode")
    env_harness.publish_decision()
    env_harness.publish_decision()

    response = env_harness.wait_future(evaluate)
    env_harness.wait_for(
        lambda: any(
            state.state == EpisodeState.EPISODE_FINISHED
            for state in env_harness.messages["state"]
        )
    )

    assert [
        (call.run_id, call.episode_id, call.frame_id)
        for call in env_harness.backend.step_calls
    ] == [("r1", "mock-000", 0)]
    assert response.accepted is True
    assert response.episode_id == "mock-000"
    assert response.scene_id == "mock-scene"
    assert response.steps == 1
    assert response.success is False
    assert response.spl == 0.25
    assert response.distance_to_goal == 0.125
    assert response.simulator_seconds == 1.5
    assert response.error == ""
    states = [
        (message.state, message.context.frame_id)
        for message in env_harness.messages["state"]
    ]
    assert states == [
        (EpisodeState.READY, 0),
        (EpisodeState.RUNNING, 0),
        (EpisodeState.ACTION_FINISHED, 1),
        (EpisodeState.EPISODE_FINISHED, 1),
    ]


def test_each_frame_has_one_full_context_and_timestamp_join(env_harness):
    evaluate = env_harness.start_episode(max_steps=1)
    env_harness.publish_decision()
    env_harness.wait_future(evaluate)
    env_harness.wait_for(
        lambda: all(
            len(env_harness.messages[name]) == 2
            for name in (
                "context",
                "rgb",
                "depth",
                "pose",
                "odometry",
                "goal",
                "goal_vector",
            )
        )
    )

    contexts = env_harness.messages["context"]
    assert [
        (
            context.contract_version,
            context.run_id,
            context.episode_id,
            context.frame_id,
            context.source_plugin_id,
        )
        for context in contexts
    ] == [
        ("luxinav.v1", "r1", "mock-000", 0, "mock"),
        ("luxinav.v1", "r1", "mock-000", 1, "mock"),
    ]
    for index, context in enumerate(contexts):
        expected_stamp = _stamp_tuple(context.stamp)
        for name in ("rgb", "depth", "pose", "odometry", "goal_vector"):
            assert _stamp_tuple(
                env_harness.messages[name][index].header.stamp
            ) == expected_stamp
        matching_states = [
            state
            for state in env_harness.messages["state"]
            if state.context.frame_id == context.frame_id
        ]
        assert matching_states
        assert all(
            _stamp_tuple(state.context.stamp) == expected_stamp
            for state in matching_states
        )
    assert [message.header.frame_id for message in env_harness.messages["rgb"]] == [
        "camera",
        "camera",
    ]
    assert [
        message.header.frame_id for message in env_harness.messages["depth"]
    ] == ["camera", "camera"]
    assert [
        message.header.frame_id for message in env_harness.messages["pose"]
    ] == ["map", "map"]
    assert [
        message.header.frame_id for message in env_harness.messages["odometry"]
    ] == ["odom", "odom"]
    assert [message.data for message in env_harness.messages["goal"]] == [
        "mock target",
        "mock target",
    ]
    assert [
        (
            message.header.frame_id,
            message.point.x,
            message.point.y,
            message.point.z,
        )
        for message in env_harness.messages["goal_vector"]
    ] == [
        ("map", 0.5, 0.0, 0.0),
        ("map", 0.5, 0.0, 0.0),
    ]
    assert env_harness.messages["rgb"][0].step == 6
    assert env_harness.messages["rgb"][0].encoding == "rgb8"
    assert bytes(env_harness.messages["rgb"][0].data) == bytes(range(6))
    assert env_harness.messages["depth"][0].step == 8
    assert env_harness.messages["depth"][0].encoding == "32FC1"
    assert bytes(env_harness.messages["depth"][0].data) == (
        b"\x00\x00\x80?\x00\x00\x00@"
    )
    assert env_harness.messages["pose"][0].pose.position.x == 0.0
    assert env_harness.messages["pose"][0].pose.position.y == 0.25
    assert env_harness.messages["pose"][0].pose.orientation.z == pytest.approx(
        math.sin(math.pi / 4)
    )
    assert env_harness.messages["pose"][0].pose.orientation.w == pytest.approx(
        math.cos(math.pi / 4)
    )
    odometry_pose = env_harness.messages["odometry"][0].pose.pose
    assert odometry_pose.position.x == 0.0
    assert odometry_pose.position.y == 0.25
    assert odometry_pose.orientation.z == pytest.approx(math.sin(math.pi / 4))
    assert odometry_pose.orientation.w == pytest.approx(math.cos(math.pi / 4))


def test_evaluate_episode_rejects_concurrent_request_but_keeps_first_alive(
    env_harness,
):
    first = env_harness.start_episode(run_id="first", max_steps=2)
    second = env_harness.evaluate_client.call_async(
        EvaluateEpisode.Request(
            run_id="second",
            episode_selector="mock-001",
            max_steps=1,
        )
    )

    second_response = env_harness.wait_future(second)

    assert second_response.accepted is False
    assert "already active" in second_response.error
    assert first.done() is False
    env_harness.publish_decision(
        run_id="first",
        episode_id="mock-000",
        kind=Decision.STOP,
    )
    first_response = env_harness.wait_future(first)
    assert first_response.accepted is True
    assert first_response.success is True
    assert len(env_harness.backend.reset_calls) == 1


def test_control_services_observe_backend_and_evaluation_deadline():
    harness = EnvHarness(evaluation_timeout_seconds=0.2)
    try:
        readiness = harness.wait_future(
            harness.readiness_client.call_async(Readiness.Request(run_id="probe"))
        )
        assert readiness.ready is True
        assert readiness.component_id == "mock"
        assert readiness.error == ""

        timed_out = harness.wait_future(harness.start_episode(), timeout=1.0)
        assert timed_out.accepted is True
        assert timed_out.steps == 0
        assert "deadline" in timed_out.error

        shutdown = harness.wait_future(
            harness.shutdown_client.call_async(Trigger.Request())
        )
        assert shutdown.success is True
        assert harness.backend.shutdown_calls == 1
    finally:
        harness.close()


def test_failed_backend_reset_is_not_reported_as_an_accepted_start():
    class FailingResetBackend(RecordingBackend):
        def reset(
            self,
            run_id,
            episode_id,
            seed,
            max_steps,
            *,
            timeout_seconds=None,
        ):
            raise RuntimeError("reset unavailable")

    harness = EnvHarness(backend=FailingResetBackend())
    try:
        future = harness.evaluate_client.call_async(
            EvaluateEpisode.Request(
                run_id="failed-run",
                episode_selector="mock-000",
                max_steps=1,
            )
        )
        response = harness.wait_future(future)
        assert response.accepted is False
        assert response.error == "reset unavailable"
        assert harness.messages["context"] == []
    finally:
        harness.close()


def test_deadline_during_step_has_one_terminal_and_no_post_terminal_publication():
    backend = BlockingBackend(block_operation="step")
    harness = EnvHarness(evaluation_timeout_seconds=0.15, backend=backend)
    try:
        evaluation = harness.start_episode(max_steps=2)
        harness.publish_decision()
        assert backend.operation_started.wait(timeout=1.0)
        response = harness.wait_future(evaluation, timeout=1.0)
        harness.wait_for(
            lambda: any(
                state.state == EpisodeState.EPISODE_FINISHED
                for state in harness.messages["state"]
            )
        )
        backend.release_operation.set()
        assert backend.operation_finished.wait(timeout=1.0)
        time.sleep(0.1)

        assert "deadline" in response.error
        assert [context.frame_id for context in harness.messages["context"]] == [0]
        assert [
            (state.state, state.context.frame_id)
            for state in harness.messages["state"]
        ] == [
            (EpisodeState.READY, 0),
            (EpisodeState.RUNNING, 0),
            (EpisodeState.EPISODE_FINISHED, 0),
        ]
        assert backend.max_active_calls == 1
        assert 0.0 < dict(backend.timeouts)["step"] <= 0.15
    finally:
        backend.release_operation.set()
        harness.close()


@pytest.mark.parametrize("operation", ["reset", "metrics"])
def test_reset_and_metrics_share_the_end_to_end_episode_deadline(operation):
    backend = BlockingBackend(block_operation=operation)
    harness = EnvHarness(evaluation_timeout_seconds=0.15, backend=backend)
    try:
        started = time.monotonic()
        evaluation = harness.evaluate_client.call_async(
            EvaluateEpisode.Request(
                run_id="slow-run",
                episode_selector="mock-000",
                max_steps=1,
            )
        )
        if operation == "metrics":
            harness.wait_for(
                lambda: any(
                    state.state == EpisodeState.READY
                    for state in harness.messages["state"]
                )
            )
            harness.publish_decision(run_id="slow-run")
        assert backend.operation_started.wait(timeout=1.0)
        completed_in_time = False
        try:
            response = harness.wait_future(evaluation, timeout=0.5)
            completed_in_time = time.monotonic() - started < 0.45
        finally:
            backend.release_operation.set()

        assert completed_in_time
        assert "deadline" in response.error
        if operation == "reset":
            assert response.accepted is False
        else:
            assert response.accepted is True
        timeout = [value for name, value in backend.timeouts if name == operation][0]
        assert 0.0 < timeout <= 0.15
    finally:
        backend.release_operation.set()
        harness.close()


def test_outstanding_step_blocks_reuse_and_shutdown_is_serialized_single_shot():
    backend = BlockingBackend(block_operation="step")
    harness = EnvHarness(evaluation_timeout_seconds=1.0, backend=backend)
    try:
        evaluation = harness.start_episode(max_steps=2)
        harness.publish_decision()
        assert backend.operation_started.wait(timeout=1.0)

        shutdown = harness.shutdown_client.call_async(Trigger.Request())
        harness.wait_for(
            lambda: any(
                state.state == EpisodeState.EPISODE_FINISHED and
                "shutdown" in state.detail
                for state in harness.messages["state"]
            ),
            timeout=0.5,
        )
        second = harness.evaluate_client.call_async(
            EvaluateEpisode.Request(
                run_id="second",
                episode_selector="mock-001",
                max_steps=1,
            )
        )
        second_response = harness.wait_future(second)
        assert second_response.accepted is False
        assert "shutting down" in second_response.error
        assert shutdown.done() is False

        backend.release_operation.set()
        shutdown_response = harness.wait_future(shutdown)
        evaluation_response = harness.wait_future(evaluation)
        repeated_shutdown = harness.wait_future(
            harness.shutdown_client.call_async(Trigger.Request())
        )

        assert shutdown_response.success is True
        assert repeated_shutdown.success is True
        assert backend.shutdown_calls == 1
        assert backend.max_active_calls == 1
        assert "shutdown" in evaluation_response.error
        harness.wait_for(
            lambda: sum(
                state.state == EpisodeState.EPISODE_FINISHED
                for state in harness.messages["state"]
            ) == 1
        )
    finally:
        backend.release_operation.set()
        harness.close()


def test_shutdown_prevents_metrics_from_starting_after_phase_change():
    backend = RecordingBackend()
    harness = EnvHarness(evaluation_timeout_seconds=1.0, backend=backend)
    metrics_waiting = threading.Event()
    release_metrics = threading.Event()
    original_backend_call = harness.env._backend_call

    def pause_before_metrics(operation, deadline, *args, **kwargs):
        if operation == backend.metrics:
            metrics_waiting.set()
            assert release_metrics.wait(timeout=1.0)
        return original_backend_call(operation, deadline, *args, **kwargs)

    harness.env._backend_call = pause_before_metrics
    try:
        evaluation = harness.start_episode(max_steps=1)
        harness.publish_decision()
        assert metrics_waiting.wait(timeout=1.0)

        shutdown = harness.shutdown_client.call_async(Trigger.Request())
        harness.wait_for(lambda: harness.env.shutdown_requested)
        shutdown_response = harness.wait_future(shutdown)
        release_metrics.set()
        evaluation_response = harness.wait_future(evaluation)

        assert shutdown_response.success is True
        assert "shutting down" in evaluation_response.error
        assert not any(name == "metrics" for name, _ in backend.timeouts)
        assert backend.shutdown_calls == 1
    finally:
        release_metrics.set()
        harness.close()


def test_failed_shutdown_is_attempted_exactly_once_across_trigger_and_close():
    class FailingShutdownBackend(RecordingBackend):
        def shutdown(self, *, timeout_seconds=None):
            self.shutdown_calls += 1
            raise RuntimeError("partial shutdown failure")

    backend = FailingShutdownBackend()
    harness = EnvHarness(backend=backend)
    try:
        first = harness.wait_future(
            harness.shutdown_client.call_async(Trigger.Request())
        )
        second = harness.wait_future(
            harness.shutdown_client.call_async(Trigger.Request())
        )
        harness.env.close()

        assert first.success is False
        assert second.success is False
        assert first.message == second.message == "partial shutdown failure"
        assert backend.shutdown_calls == 1
    finally:
        harness.close()


def test_shutdown_lock_timeout_can_retry_before_backend_invocation():
    backend = RecordingBackend()
    harness = EnvHarness(evaluation_timeout_seconds=0.1, backend=backend)
    harness.env._backend_lock.acquire()
    try:
        first = harness.wait_future(
            harness.shutdown_client.call_async(Trigger.Request())
        )
        assert first.success is False
        assert "deadline" in first.message
        assert backend.shutdown_calls == 0
    finally:
        harness.env._backend_lock.release()

    try:
        second = harness.wait_future(
            harness.shutdown_client.call_async(Trigger.Request())
        )
        assert second.success is True
        assert backend.shutdown_calls == 1
    finally:
        harness.close()


def test_terminal_reset_returns_without_ready_or_deadline_wait():
    class TerminalResetBackend(RecordingBackend):
        def reset(
            self,
            run_id,
            episode_id,
            seed,
            max_steps,
            *,
            timeout_seconds=None,
        ):
            observation = super().reset(
                run_id,
                episode_id,
                seed,
                max_steps,
                timeout_seconds=timeout_seconds,
            )
            return _observation(
                observation.run_id,
                observation.episode_id,
                observation.frame_id,
                terminal=True,
            )

    harness = EnvHarness(
        evaluation_timeout_seconds=0.5,
        backend=TerminalResetBackend(),
    )
    try:
        started = time.monotonic()
        response = harness.wait_future(
            harness.evaluate_client.call_async(
                EvaluateEpisode.Request(
                    run_id="terminal-run",
                    episode_selector="mock-000",
                    max_steps=1,
                )
            )
        )
        harness.wait_for(lambda: len(harness.messages["state"]) == 1)

        assert time.monotonic() - started < 0.3
        assert response.accepted is True
        assert [
            (state.state, state.context.frame_id)
            for state in harness.messages["state"]
        ] == [(EpisodeState.EPISODE_FINISHED, 0)]
    finally:
        harness.close()
