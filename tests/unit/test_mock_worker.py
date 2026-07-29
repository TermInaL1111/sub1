import base64
import json
from pathlib import Path

import pytest

from workers.mock_worker.state_machine import MockStateMachine
from luxinav_sim.backend_protocol import DecisionPayload, ProtocolValidationError


FIXTURE = Path(__file__).parents[1] / "fixtures" / "mock_episode_seed7.json"


def run_episode(seed, actions):
    machine = MockStateMachine()
    observation = machine.reset("run-id", "mock-000", seed=seed, max_steps=3)
    frames = [observation.to_wire()]
    for frame_id, action in enumerate(actions):
        decision = DecisionPayload.from_wire(
            {
                "run_id": "run-id",
                "episode_id": "mock-000",
                "frame_id": frame_id,
                **action,
            }
        )
        frames.append(machine.step(decision).to_wire())
    return frames


def checkpoint(frame):
    result = {
        "frame_id": frame["frame_id"],
        "pose": frame["pose"],
        "state": frame["state"],
    }
    if "metrics" in frame:
        result["metrics"] = frame["metrics"]
    return result


def test_same_seed_and_actions_produce_identical_episode_checkpoint():
    actions = [
        {"kind": "continuous_control", "linear_x": 0.5, "angular_z": 0.0},
        {"kind": "continuous_control", "linear_x": 0.0, "angular_z": 0.174532925},
        {"kind": "stop"},
    ]

    first = run_episode(seed=7, actions=actions)
    second = run_episode(seed=7, actions=actions)

    assert first == second
    assert first[-1]["state"] == "EPISODE_FINISHED"
    assert first[-1]["metrics"]["success"] is True
    assert [checkpoint(frame) for frame in first] == json.loads(FIXTURE.read_text(encoding="utf-8"))


def test_each_encoded_sensor_payload_has_its_own_validated_size():
    frames = run_episode(seed=7, actions=[])
    frame = frames[0]

    assert len(base64.b64decode(frame["rgb"]["data_b64"], validate=True)) == 4 * 4 * 3
    assert len(base64.b64decode(frame["depth"]["data_b64"], validate=True)) == 4 * 4 * 4


@pytest.mark.parametrize(
    ("action", "expected_pose"),
    [
        ({"kind": "discrete_action", "action": "forward"}, {"x": 0.25, "y": 0.0, "yaw": 0.0}),
        ({"kind": "discrete_action", "action": "turn_left"}, {"x": 0.0, "y": 0.0, "yaw": 0.17453292519943295}),
        ({"kind": "discrete_action", "action": "turn_right"}, {"x": 0.0, "y": 0.0, "yaw": -0.17453292519943295}),
    ],
)
def test_discrete_actions_use_verified_motion_constants(action, expected_pose):
    machine = MockStateMachine()
    machine.reset("run-id", "mock-000", seed=7, max_steps=5)

    result = machine.step(
        DecisionPayload.from_wire(
            {"run_id": "run-id", "episode_id": "mock-000", "frame_id": 0, **action}
        )
    )

    assert result.observation.to_wire()["pose"] == expected_pose


def test_timeout_finishes_episode_at_max_steps_without_success():
    machine = MockStateMachine()
    machine.reset("run-id", "mock-000", seed=7, max_steps=1)

    result = machine.step(
        DecisionPayload.from_wire(
            {
                "run_id": "run-id",
                "episode_id": "mock-000",
                "frame_id": 0,
                "kind": "continuous_control",
                "linear_x": 0.0,
                "angular_z": 0.0,
            }
        )
    )

    assert result.to_wire()["state"] == "EPISODE_FINISHED"
    assert result.to_wire()["metrics"] == {"steps": 1, "success": False}


def test_rejects_invalid_decisions_without_advancing_the_frame():
    machine = MockStateMachine()
    machine.reset("run-id", "mock-000", seed=7, max_steps=3)

    with pytest.raises(ProtocolValidationError, match="linear_x"):
        DecisionPayload.from_wire(
            {
                "run_id": "run-id",
                "episode_id": "mock-000",
                "frame_id": 0,
                "kind": "continuous_control",
                "angular_z": 0.0,
            }
        )

    result = machine.step(
        DecisionPayload.from_wire(
            {
                "run_id": "run-id",
                "episode_id": "mock-000",
                "frame_id": 0,
                "kind": "stop",
            }
        )
    )
    assert result.observation.frame_id == 1
