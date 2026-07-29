"""Pure deterministic transition model used by the mock worker."""

from __future__ import annotations

import math
import random
import struct
from typing import Any

from luxinav_sim.backend_protocol import (
    DecisionPayload,
    ImagePayload,
    Observation,
    ProtocolValidationError,
    StepResult,
)


class SessionConflictError(RuntimeError):
    def __init__(self, error: str, detail: str, expected_frame_id: int | None) -> None:
        self.error = error
        self.detail = detail
        self.expected_frame_id = expected_frame_id
        super().__init__(detail)


class MockStateMachine:
    """One reset-scoped mock episode with deterministic RGB-D observations."""

    _GOAL_X = 0.5
    _SUCCESS_RADIUS = 0.20
    _DISCRETE_FORWARD = 0.25
    _DISCRETE_TURN = math.radians(10)

    def __init__(self) -> None:
        self._run_id: str | None = None
        self._episode_id: str | None = None
        self._seed: int | None = None
        self._max_steps = 0
        self._frame_id = 0
        self._steps = 0
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._finished = False
        self._success = False

    def reset(self, run_id: str, episode_id: str, seed: int, max_steps: int) -> Observation:
        if not isinstance(run_id, str) or not run_id:
            raise ProtocolValidationError("run_id must be a non-empty string")
        if not isinstance(episode_id, str) or not episode_id:
            raise ProtocolValidationError("episode_id must be a non-empty string")
        if isinstance(seed, bool) or not isinstance(seed, int):
            raise ProtocolValidationError("seed must be an integer")
        if isinstance(max_steps, bool) or not isinstance(max_steps, int) or max_steps < 1:
            raise ProtocolValidationError("max_steps must be a positive integer")
        self._run_id = run_id
        self._episode_id = episode_id
        self._seed = seed
        self._max_steps = max_steps
        self._frame_id = 0
        self._steps = 0
        self._x = self._y = self._yaw = 0.0
        self._finished = self._success = False
        return self._observation()

    def step(self, decision: DecisionPayload) -> StepResult:
        self._validate_session(decision)
        if self._finished:
            raise SessionConflictError("episode_finished", "active episode is already finished", self._frame_id)
        if decision.kind == "continuous_control":
            self._move(decision.linear_x or 0.0, decision.angular_z or 0.0)
        elif decision.kind == "discrete_action":
            if decision.action == "forward":
                self._move(self._DISCRETE_FORWARD, 0.0)
            elif decision.action == "turn_left":
                self._move(0.0, self._DISCRETE_TURN)
            else:
                self._move(0.0, -self._DISCRETE_TURN)

        self._steps += 1
        self._frame_id += 1
        if decision.kind == "stop":
            self._success = self._distance_to_goal() <= self._SUCCESS_RADIUS
            self._finished = True
        elif self._steps >= self._max_steps:
            self._finished = True
        return StepResult(self._observation(), {"steps": self._steps, "success": self._success})

    def metrics(self) -> dict[str, Any]:
        self._require_session()
        return {"steps": self._steps, "success": self._success}

    def _validate_session(self, decision: DecisionPayload) -> None:
        self._require_session()
        if decision.run_id != self._run_id:
            raise SessionConflictError(
                "session_mismatch", "decision run_id does not match active session", self._frame_id
            )
        if decision.episode_id != self._episode_id:
            raise SessionConflictError(
                "session_mismatch", "decision episode_id does not match active session", self._frame_id
            )
        if decision.frame_id != self._frame_id:
            raise SessionConflictError(
                "frame_mismatch", "decision frame_id does not match expected frame", self._frame_id
            )

    def _require_session(self) -> None:
        if self._run_id is None:
            raise SessionConflictError("session_missing", "no active session", None)

    def _move(self, linear_x: float, angular_z: float) -> None:
        self._x += linear_x * math.cos(self._yaw)
        self._y += linear_x * math.sin(self._yaw)
        self._yaw += angular_z

    def _distance_to_goal(self) -> float:
        return math.hypot(self._GOAL_X - self._x, -self._y)

    def _observation(self) -> Observation:
        assert self._run_id is not None and self._episode_id is not None and self._seed is not None
        rng = random.Random(f"{self._seed}:{self._frame_id}")
        rgb = bytes(rng.randrange(256) for _ in range(4 * 4 * 3))
        depth = struct.pack("<16f", *(rng.random() for _ in range(4 * 4)))
        return Observation.from_wire(
            {
                "run_id": self._run_id,
                "episode_id": self._episode_id,
                "frame_id": self._frame_id,
                "rgb": ImagePayload(4, 4, "rgb8", rgb).to_wire(),
                "depth": ImagePayload(4, 4, "32FC1", depth).to_wire(),
                "pose": {"x": self._x, "y": self._y, "yaw": self._yaw},
                "goal": {"text": "mock target", "x": self._GOAL_X, "y": 0.0},
                "state": "EPISODE_FINISHED" if self._finished else "READY",
            }
        )
