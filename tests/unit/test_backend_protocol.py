import base64
import json
import os
from pathlib import Path
import select
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest

from luxinav_sim.backend_protocol import (
    DecisionPayload,
    EnvironmentBackend,
    EpisodeMetrics,
    ImagePayload,
    Observation,
    ProtocolValidationError,
    StepResult,
)
from luxinav_sim.http_backend import (
    BackendHttpError,
    BackendTransportError,
    HttpEnvironmentBackend,
)
from workers.mock_worker.server import create_server
from workers.mock_worker.state_machine import MockStateMachine


class ConformingFakeBackend:
    def __init__(self):
        self.observation = Observation(
            run_id="run-a",
            episode_id="mock-000",
            frame_id=0,
            rgb=ImagePayload(1, 1, "rgb8", b"\x00" * 3),
            depth=ImagePayload(1, 1, "32FC1", b"\x00" * 4),
            pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
            goal={"text": "mock target", "x": 0.5, "y": 0.0},
            state="READY",
        )
        self.episode_metrics = EpisodeMetrics(
            scene_id="mock-scene",
            success=False,
            spl=0.0,
            distance_to_goal=0.5,
            steps=0,
            simulator_seconds=0.0,
        )

    def health(self, *, timeout_seconds=None):
        return {"status": "ok"}

    def reset(self, run_id, episode_id, seed, max_steps, *, timeout_seconds=None):
        return self.observation

    def step(self, decision, *, timeout_seconds=None):
        return StepResult(self.observation, self.episode_metrics)

    def metrics(self, *, timeout_seconds=None):
        return self.episode_metrics

    def shutdown(self, *, timeout_seconds=None):
        return {"status": "shutting_down"}


def test_environment_backend_protocol_accepts_complete_fake_and_http_client():
    fake = ConformingFakeBackend()
    http = HttpEnvironmentBackend("http://127.0.0.1:1")
    decision = DecisionPayload("run-a", "mock-000", 0, "stop")

    assert isinstance(fake, EnvironmentBackend)
    assert isinstance(http, EnvironmentBackend)
    assert fake.health(timeout_seconds=0.5) == {"status": "ok"}
    assert (
        fake.reset(
            "run-a", "mock-000", seed=7, max_steps=3, timeout_seconds=0.5
        )
        is fake.observation
    )
    assert fake.step(decision, timeout_seconds=0.5).observation is fake.observation
    assert fake.metrics(timeout_seconds=0.5) == {
        "scene_id": "mock-scene",
        "success": False,
        "spl": 0.0,
        "distance_to_goal": 0.5,
        "steps": 0,
        "simulator_seconds": 0.0,
    }
    assert fake.shutdown(timeout_seconds=0.5) == {"status": "shutting_down"}


def test_episode_metrics_validates_and_preserves_the_complete_evaluation_contract():
    metrics = EpisodeMetrics.from_wire(
        {
            "scene_id": "mock-scene",
            "success": True,
            "spl": 0.625,
            "distance_to_goal": 0.125,
            "steps": 4,
            "simulator_seconds": 4.0,
        }
    )

    assert metrics.to_wire() == {
        "scene_id": "mock-scene",
        "success": True,
        "spl": 0.625,
        "distance_to_goal": 0.125,
        "steps": 4,
        "simulator_seconds": 4.0,
    }


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("scene_id", ""),
        ("success", 1),
        ("spl", 1.01),
        ("distance_to_goal", -0.01),
        ("steps", -1),
        ("simulator_seconds", -0.01),
    ],
)
def test_episode_metrics_rejects_invalid_evaluation_values(field, value):
    wire = {
        "scene_id": "mock-scene",
        "success": True,
        "spl": 1.0,
        "distance_to_goal": 0.0,
        "steps": 3,
        "simulator_seconds": 3.0,
    }
    wire[field] = value

    with pytest.raises(ProtocolValidationError, match=field):
        EpisodeMetrics.from_wire(wire)


@pytest.fixture
def backend():
    server = create_server(port=0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    client = HttpEnvironmentBackend(f"http://127.0.0.1:{server.server_port}")
    try:
        yield client
    finally:
        client.shutdown()
        thread.join(timeout=2)
        server.server_close()
        assert not thread.is_alive()


class SlowHandler(BaseHTTPRequestHandler):
    def do_GET(self):  # noqa: N802
        self._delay_response()

    def do_POST(self):  # noqa: N802
        self._delay_response()

    def _delay_response(self):
        time.sleep(0.2)

    def log_message(self, format, *args):
        return


class DaemonThreadingHTTPServer(ThreadingHTTPServer):
    daemon_threads = True


@pytest.fixture
def slow_backend_url():
    server = DaemonThreadingHTTPServer(("127.0.0.1", 0), SlowHandler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield f"http://127.0.0.1:{server.server_port}"
    finally:
        server.shutdown()
        thread.join(timeout=2)
        server.server_close()
        assert not thread.is_alive()


@pytest.mark.parametrize(
    "operation",
    [
        lambda backend: backend.health(timeout_seconds=0.01),
        lambda backend: backend.reset(
            "run-a", "mock-000", seed=7, max_steps=3, timeout_seconds=0.01
        ),
        lambda backend: backend.step(
            DecisionPayload("run-a", "mock-000", 0, "stop"),
            timeout_seconds=0.01,
        ),
        lambda backend: backend.metrics(timeout_seconds=0.01),
        lambda backend: backend.shutdown(timeout_seconds=0.01),
    ],
)
def test_each_http_operation_honors_its_per_call_timeout(slow_backend_url, operation):
    backend = HttpEnvironmentBackend(slow_backend_url, timeout_seconds=1.0)

    with pytest.raises(BackendTransportError, match="timed out"):
        operation(backend)


def test_per_call_timeout_cannot_exceed_the_configured_default(slow_backend_url):
    backend = HttpEnvironmentBackend(slow_backend_url, timeout_seconds=0.01)

    with pytest.raises(BackendTransportError, match="timed out"):
        backend.health(timeout_seconds=1.0)


def test_real_http_round_trip_exposes_all_environment_operations(backend):
    assert backend.health() == {"status": "ok"}
    reset = backend.reset("run-a", "mock-000", seed=7, max_steps=3)

    step = backend.step(
        {
            "run_id": "run-a",
            "episode_id": "mock-000",
            "frame_id": reset.frame_id,
            "kind": "continuous_control",
            "linear_x": 0.5,
            "angular_z": 0.0,
        }
    )

    assert step.observation.frame_id == 1
    assert backend.metrics() == {
        "scene_id": "mock-scene",
        "success": False,
        "spl": 0.0,
        "distance_to_goal": 0.0,
        "steps": 1,
        "simulator_seconds": 1.0,
    }


class PauseAfterFirstValidationStateMachine(MockStateMachine):
    def __init__(self):
        super().__init__()
        self.first_validated = threading.Event()
        self.second_validated = threading.Event()
        self.release_first = threading.Event()
        self._validation_count = 0
        self._validation_count_lock = threading.Lock()

    def _validate_session(self, decision):
        super()._validate_session(decision)
        with self._validation_count_lock:
            self._validation_count += 1
            validation_count = self._validation_count
        if validation_count == 1:
            self.first_validated.set()
            assert self.release_first.wait(timeout=2)
        else:
            self.second_validated.set()


def test_duplicate_concurrent_steps_are_serialized_and_one_is_rejected():
    state_machine = PauseAfterFirstValidationStateMachine()
    server = create_server(port=0, state_machine=state_machine)
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    client = HttpEnvironmentBackend(f"http://127.0.0.1:{server.server_port}")
    observation = client.reset("run-a", "mock-000", seed=7, max_steps=3)
    decision = DecisionPayload(
        "run-a",
        "mock-000",
        observation.frame_id,
        "continuous_control",
        linear_x=0.5,
        angular_z=0.0,
    )
    outcomes = []
    outcomes_lock = threading.Lock()

    def submit_step():
        try:
            outcome = client.step(decision)
        except BackendHttpError as error:
            outcome = error
        with outcomes_lock:
            outcomes.append(outcome)

    first = threading.Thread(target=submit_step)
    second = threading.Thread(target=submit_step)
    try:
        first.start()
        assert state_machine.first_validated.wait(timeout=2)
        second.start()
        state_machine.second_validated.wait(timeout=1)
        state_machine.release_first.set()
        first.join(timeout=2)
        second.join(timeout=2)

        assert not first.is_alive()
        assert not second.is_alive()
        assert len([value for value in outcomes if isinstance(value, StepResult)]) == 1
        conflicts = [
            value for value in outcomes if isinstance(value, BackendHttpError)
        ]
        assert len(conflicts) == 1
        assert conflicts[0].status == 409
        assert conflicts[0].payload["error"] == "frame_mismatch"
        assert client.metrics()["steps"] == 1
    finally:
        state_machine.release_first.set()
        client.shutdown()
        server_thread.join(timeout=2)
        server.server_close()
        assert not server_thread.is_alive()


def test_http_conflict_reports_expected_frame_for_stale_or_foreign_decisions(backend):
    reset = backend.reset("run-a", "mock-000", seed=7, max_steps=3)
    backend.step(
        {
            "run_id": "run-a",
            "episode_id": "mock-000",
            "frame_id": reset.frame_id,
            "kind": "stop",
        }
    )

    with pytest.raises(BackendHttpError) as error:
        backend.step(
            {
                "run_id": "another-run",
                "episode_id": "mock-000",
                "frame_id": 0,
                "kind": "stop",
            }
        )

    assert error.value.status == 409
    assert error.value.payload == {
        "error": "session_mismatch",
        "detail": "decision run_id does not match active session",
        "expected_frame_id": 1,
    }


@pytest.mark.parametrize("missing_field", ["run_id", "episode_id", "frame_id"])
def test_http_rejects_missing_session_identifiers_with_a_conflict(backend, missing_field):
    reset = backend.reset("run-a", "mock-000", seed=7, max_steps=3)
    payload = {
        "run_id": "run-a",
        "episode_id": "mock-000",
        "frame_id": reset.frame_id,
        "kind": "stop",
    }
    del payload[missing_field]
    request = Request(
        f"{backend._base_url}/step",
        data=json.dumps(payload).encode("utf-8"),
        method="POST",
        headers={"Content-Type": "application/json"},
    )

    with pytest.raises(HTTPError) as error:
        urlopen(request)

    assert error.value.code == 409
    assert json.loads(error.value.read()) == {
        "error": "session_mismatch",
        "detail": f"decision {missing_field} is required",
        "expected_frame_id": 0,
    }


@pytest.mark.parametrize(
    ("field", "value", "expected"),
    [
        ("run_id", "other-run", {"error": "session_mismatch", "detail": "decision run_id does not match active session", "expected_frame_id": 0}),
        ("episode_id", "other-episode", {"error": "session_mismatch", "detail": "decision episode_id does not match active session", "expected_frame_id": 0}),
        ("frame_id", 1, {"error": "frame_mismatch", "detail": "decision frame_id does not match expected frame", "expected_frame_id": 0}),
    ],
)
def test_http_rejects_mismatched_session_identifiers_with_complete_conflicts(
    backend, field, value, expected
):
    reset = backend.reset("run-a", "mock-000", seed=7, max_steps=3)
    payload = {"run_id": "run-a", "episode_id": "mock-000", "frame_id": reset.frame_id, "kind": "stop"}
    payload[field] = value

    with pytest.raises(BackendHttpError) as error:
        backend.step(payload)

    assert error.value.status == 409
    assert error.value.payload == expected


def test_worker_process_binds_loopback_and_exits_after_shutdown():
    root = Path(__file__).parents[2]
    env = {**os.environ, "PYTHONPATH": f"{root}:{root / 'ros2_ws/src/luxinav_sim'}"}
    process = subprocess.Popen(
        [sys.executable, "-u", "-m", "workers.mock_worker.server"],
        cwd=root,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    try:
        ready, _, _ = select.select([process.stdout], [], [], 1)
        assert ready, "worker did not announce its listening address"
        address = json.loads(process.stdout.readline())
        client = HttpEnvironmentBackend(f"http://{address['host']}:{address['port']}")
        assert address["host"] == "127.0.0.1"
        assert client.health() == {"status": "ok"}
        assert client.shutdown() == {"status": "shutting_down"}
        assert process.wait(timeout=2) == 0
    finally:
        if process.poll() is None:
            process.kill()
            process.wait(timeout=2)


def test_reset_starts_an_isolated_session_and_invalidates_previous_run(backend):
    first = backend.reset("run-a", "mock-000", seed=7, max_steps=3)
    backend.step(
        {
            "run_id": "run-a",
            "episode_id": "mock-000",
            "frame_id": first.frame_id,
            "kind": "continuous_control",
            "linear_x": 0.5,
            "angular_z": 0.0,
        }
    )
    second = backend.reset("run-b", "mock-001", seed=7, max_steps=3)

    assert second.frame_id == 0
    assert second.pose == {"x": 0.0, "y": 0.0, "yaw": 0.0}
    with pytest.raises(BackendHttpError, match="session_mismatch"):
        backend.step(
            {
                "run_id": "run-a",
                "episode_id": "mock-000",
                "frame_id": 1,
                "kind": "stop",
            }
        )


@pytest.mark.parametrize(
    "wire",
    [
        {"width": 4, "height": 4, "encoding": "rgb8", "data_b64": base64.b64encode(b"x" * 47).decode()},
        {"width": 4, "height": 4, "encoding": "32FC1", "data_b64": base64.b64encode(b"x" * 63).decode()},
    ],
)
def test_image_protocol_rejects_rgb_and_depth_size_errors_independently(wire):
    with pytest.raises(ProtocolValidationError, match="byte length"):
        ImagePayload.from_wire(wire)
