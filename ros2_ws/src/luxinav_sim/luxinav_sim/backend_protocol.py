"""Simulator-neutral wire values for LuxiNav environment backends."""

from __future__ import annotations

import base64
from dataclasses import dataclass
from types import MappingProxyType
from typing import Any, Mapping, Protocol, runtime_checkable


class ProtocolValidationError(ValueError):
    """A wire value does not conform to the environment protocol."""


def _mapping(value: Any, field: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ProtocolValidationError(f"{field} must be a mapping")
    return value


def _string(value: Any, field: str) -> str:
    if not isinstance(value, str) or not value:
        raise ProtocolValidationError(f"{field} must be a non-empty string")
    return value


def _integer(value: Any, field: str, minimum: int = 0) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise ProtocolValidationError(f"{field} must be an integer >= {minimum}")
    return value


def _number(value: Any, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ProtocolValidationError(f"{field} must be a number")
    return float(value)


def _required(value: Mapping[str, Any], names: set[str], context: str) -> None:
    missing = sorted(names.difference(value))
    if missing:
        raise ProtocolValidationError(f"{context} missing fields: {', '.join(missing)}")


@dataclass(frozen=True)
class ImagePayload:
    width: int
    height: int
    encoding: str
    data: bytes

    @classmethod
    def from_wire(cls, value: Any) -> "ImagePayload":
        wire = _mapping(value, "image")
        _required(wire, {"width", "height", "encoding", "data_b64"}, "image")
        width = _integer(wire["width"], "image.width", minimum=1)
        height = _integer(wire["height"], "image.height", minimum=1)
        encoding = _string(wire["encoding"], "image.encoding")
        if encoding not in {"rgb8", "32FC1"}:
            raise ProtocolValidationError(f"unsupported image.encoding: {encoding}")
        try:
            data = base64.b64decode(_string(wire["data_b64"], "image.data_b64"), validate=True)
        except ValueError as error:
            raise ProtocolValidationError("image.data_b64 must be valid base64") from error
        bytes_per_pixel = 3 if encoding == "rgb8" else 4
        expected_size = width * height * bytes_per_pixel
        if len(data) != expected_size:
            raise ProtocolValidationError(
                f"image byte length must be {expected_size}, got {len(data)}"
            )
        return cls(width=width, height=height, encoding=encoding, data=data)

    def to_wire(self) -> dict[str, Any]:
        return {
            "width": self.width,
            "height": self.height,
            "encoding": self.encoding,
            "data_b64": base64.b64encode(self.data).decode("ascii"),
        }


@dataclass(frozen=True)
class DecisionPayload:
    run_id: str
    episode_id: str
    frame_id: int
    kind: str
    linear_x: float | None = None
    angular_z: float | None = None
    action: str | None = None

    @classmethod
    def from_wire(cls, value: Any) -> "DecisionPayload":
        wire = _mapping(value, "decision")
        _required(wire, {"run_id", "episode_id", "frame_id", "kind"}, "decision")
        kind = _string(wire["kind"], "decision.kind")
        decision = cls(
            run_id=_string(wire["run_id"], "decision.run_id"),
            episode_id=_string(wire["episode_id"], "decision.episode_id"),
            frame_id=_integer(wire["frame_id"], "decision.frame_id"),
            kind=kind,
            linear_x=_number(wire["linear_x"], "decision.linear_x")
            if "linear_x" in wire
            else None,
            angular_z=_number(wire["angular_z"], "decision.angular_z")
            if "angular_z" in wire
            else None,
            action=_string(wire["action"], "decision.action") if "action" in wire else None,
        )
        if kind == "continuous_control":
            if decision.linear_x is None:
                raise ProtocolValidationError("decision.linear_x is required for continuous_control")
            if decision.angular_z is None:
                raise ProtocolValidationError("decision.angular_z is required for continuous_control")
        elif kind == "discrete_action":
            if decision.action not in {"forward", "turn_left", "turn_right"}:
                raise ProtocolValidationError(
                    "decision.action must be forward, turn_left, or turn_right"
                )
        elif kind != "stop":
            raise ProtocolValidationError(f"unsupported decision.kind: {kind}")
        return decision

    def to_wire(self) -> dict[str, Any]:
        result: dict[str, Any] = {
            "run_id": self.run_id,
            "episode_id": self.episode_id,
            "frame_id": self.frame_id,
            "kind": self.kind,
        }
        if self.kind == "continuous_control":
            result.update({"linear_x": self.linear_x, "angular_z": self.angular_z})
        if self.kind == "discrete_action":
            result["action"] = self.action
        return result


@dataclass(frozen=True)
class Observation:
    run_id: str
    episode_id: str
    frame_id: int
    rgb: ImagePayload
    depth: ImagePayload
    pose: Mapping[str, float]
    goal: Mapping[str, Any]
    state: str

    @classmethod
    def from_wire(cls, value: Any) -> "Observation":
        wire = _mapping(value, "observation")
        _required(
            wire,
            {"run_id", "episode_id", "frame_id", "rgb", "depth", "pose", "goal", "state"},
            "observation",
        )
        pose_wire = _mapping(wire["pose"], "observation.pose")
        goal_wire = _mapping(wire["goal"], "observation.goal")
        _required(pose_wire, {"x", "y", "yaw"}, "observation.pose")
        _required(goal_wire, {"text", "x", "y"}, "observation.goal")
        state = _string(wire["state"], "observation.state")
        if state not in {"READY", "EPISODE_FINISHED"}:
            raise ProtocolValidationError(f"unsupported observation.state: {state}")
        return cls(
            run_id=_string(wire["run_id"], "observation.run_id"),
            episode_id=_string(wire["episode_id"], "observation.episode_id"),
            frame_id=_integer(wire["frame_id"], "observation.frame_id"),
            rgb=ImagePayload.from_wire(wire["rgb"]),
            depth=ImagePayload.from_wire(wire["depth"]),
            pose=MappingProxyType({key: _number(pose_wire[key], f"observation.pose.{key}") for key in ("x", "y", "yaw")} ),
            goal=MappingProxyType(
                {
                    "text": _string(goal_wire["text"], "observation.goal.text"),
                    "x": _number(goal_wire["x"], "observation.goal.x"),
                    "y": _number(goal_wire["y"], "observation.goal.y"),
                }
            ),
            state=state,
        )

    def to_wire(self) -> dict[str, Any]:
        return {
            "run_id": self.run_id,
            "episode_id": self.episode_id,
            "frame_id": self.frame_id,
            "rgb": self.rgb.to_wire(),
            "depth": self.depth.to_wire(),
            "pose": dict(self.pose),
            "goal": dict(self.goal),
            "state": self.state,
        }


@dataclass(frozen=True)
class StepResult:
    observation: Observation
    metrics: Mapping[str, Any]

    @classmethod
    def from_wire(cls, value: Any) -> "StepResult":
        wire = _mapping(value, "step result")
        _required(wire, {"metrics"}, "step result")
        metrics = _mapping(wire["metrics"], "step result.metrics")
        _required(metrics, {"steps", "success"}, "step result.metrics")
        steps = _integer(metrics["steps"], "step result.metrics.steps")
        if not isinstance(metrics["success"], bool):
            raise ProtocolValidationError("step result.metrics.success must be a boolean")
        observation_wire = {key: value for key, value in wire.items() if key != "metrics"}
        return cls(
            observation=Observation.from_wire(observation_wire),
            metrics=MappingProxyType({"steps": steps, "success": metrics["success"]}),
        )

    def to_wire(self) -> dict[str, Any]:
        return {**self.observation.to_wire(), "metrics": dict(self.metrics)}


@runtime_checkable
class EnvironmentBackend(Protocol):
    """Structural boundary implemented by simulator environment backends."""

    def health(self) -> Mapping[str, Any]: ...

    def reset(
        self, run_id: str, episode_id: str, seed: int, max_steps: int
    ) -> Observation: ...

    def step(self, decision: DecisionPayload) -> StepResult: ...

    def metrics(self) -> Mapping[str, Any]: ...

    def shutdown(self) -> Mapping[str, Any]: ...
