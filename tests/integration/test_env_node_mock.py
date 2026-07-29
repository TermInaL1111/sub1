import threading
import time

from luxinav_interfaces.msg import Decision, EpisodeState, RunContext
from luxinav_interfaces.srv import EvaluateEpisode
from luxinav_sim.env_node import EnvNode
from luxinav_sim.http_backend import HttpEnvironmentBackend
import rclpy
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_srvs.srv import Trigger
from workers.mock_worker.server import create_server
from workers.mock_worker.state_machine import MockStateMachine


class RecordingMockStateMachine(MockStateMachine):
    def __init__(self):
        super().__init__()
        self.reset_calls = []
        self.step_calls = []

    def reset(self, run_id, episode_id, seed, max_steps):
        self.reset_calls.append((run_id, episode_id, seed, max_steps))
        return super().reset(run_id, episode_id, seed, max_steps)

    def step(self, decision):
        self.step_calls.append(decision)
        return super().step(decision)


def wait_for(predicate, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError("condition was not observed before the deadline")


def test_real_mock_http_backend_drives_one_ros_frame_transition():
    machine = RecordingMockStateMachine()
    server = create_server(port=0, state_machine=machine)
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    backend = HttpEnvironmentBackend(
        f"http://127.0.0.1:{server.server_port}"
    )

    context = Context()
    rclpy.init(context=context)
    env = EnvNode(
        backend,
        context=context,
        source_plugin_id="mock",
        evaluation_timeout_seconds=2.0,
    )
    probe = Node("env_mock_integration_probe", context=context)
    reliable = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
    contexts = []
    states = []
    context_sub = probe.create_subscription(
        RunContext,
        "/luxinav/observation/context",
        contexts.append,
        reliable,
    )
    state_sub = probe.create_subscription(
        EpisodeState,
        "/luxinav/episode/state",
        states.append,
        reliable,
    )
    decision_pub = probe.create_publisher(
        Decision, "/luxinav/decision", reliable
    )
    evaluate_client = probe.create_client(
        EvaluateEpisode, "/luxinav/env/evaluate_episode"
    )
    shutdown_client = probe.create_client(Trigger, "/luxinav/env/shutdown")
    executor = MultiThreadedExecutor(num_threads=4, context=context)
    executor.add_node(env)
    executor.add_node(probe)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        assert evaluate_client.wait_for_service(timeout_sec=2.0)
        assert shutdown_client.wait_for_service(timeout_sec=2.0)
        evaluation = evaluate_client.call_async(
            EvaluateEpisode.Request(
                run_id="integration-run",
                episode_selector="mock-000",
                max_steps=1,
            )
        )
        wait_for(
            lambda: any(state.state == EpisodeState.READY for state in states)
        )
        assert evaluation.done() is False

        def publish(run_id, episode_id, frame_id):
            message = Decision(kind=Decision.CONTINUOUS_CONTROL)
            message.context.contract_version = "luxinav.v1"
            message.context.run_id = run_id
            message.context.episode_id = episode_id
            message.context.frame_id = frame_id
            message.context.source_plugin_id = "integration-agent"
            decision_pub.publish(message)

        publish("integration-run", "mock-000", 99)
        publish("foreign-run", "mock-000", 0)
        publish("integration-run", "mock-000", 0)
        publish("integration-run", "mock-000", 0)

        wait_for(evaluation.done)
        response = evaluation.result()
        wait_for(lambda: [item.frame_id for item in contexts] == [0, 1])
        wait_for(
            lambda: any(
                state.state == EpisodeState.EPISODE_FINISHED for state in states
            )
        )

        assert machine.reset_calls == [
            ("integration-run", "mock-000", 0, 1)
        ]
        assert [
            (call.run_id, call.episode_id, call.frame_id)
            for call in machine.step_calls
        ] == [("integration-run", "mock-000", 0)]
        assert backend.metrics() == {
            "scene_id": "mock-scene",
            "success": False,
            "spl": 0.0,
            "distance_to_goal": 0.5,
            "steps": 1,
            "simulator_seconds": 1.0,
        }
        assert response.accepted is True
        assert response.episode_id == "mock-000"
        assert response.steps == 1
        assert response.success is False
        assert response.error == ""
        assert [
            (state.state, state.context.frame_id) for state in states
        ] == [
            (EpisodeState.READY, 0),
            (EpisodeState.RUNNING, 0),
            (EpisodeState.ACTION_FINISHED, 1),
            (EpisodeState.EPISODE_FINISHED, 1),
        ]

        shutdown = shutdown_client.call_async(Trigger.Request())
        wait_for(shutdown.done)
        assert shutdown.result().success is True
        server_thread.join(timeout=2.0)
        assert not server_thread.is_alive()
    finally:
        executor.shutdown(timeout_sec=2.0)
        executor_thread.join(timeout=2.0)
        probe.destroy_subscription(context_sub)
        probe.destroy_subscription(state_sub)
        probe.destroy_node()
        env.destroy_node()
        rclpy.shutdown(context=context)
        if server_thread.is_alive():
            server.shutdown()
            server_thread.join(timeout=2.0)
        server.server_close()
        assert not executor_thread.is_alive()
        assert not server_thread.is_alive()
