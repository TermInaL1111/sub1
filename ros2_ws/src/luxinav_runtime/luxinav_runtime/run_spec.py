"""Canonical, immutable run specifications derived from plugin IDs."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
import json
from secrets import token_hex
from types import MappingProxyType
from typing import Any, Mapping

from .catalog import Catalog


CONTRACT_VERSION = "luxinav.v1"


class CompatibilityError(ValueError):
    def __init__(self, payload: Mapping[str, Any]) -> None:
        self.payload = MappingProxyType(dict(payload))
        super().__init__(json.dumps(dict(self.payload), sort_keys=True, separators=(",", ":")))


class CapabilityError(CompatibilityError):
    """A selected component needs observations its environment does not provide."""


class DecisionCompatibilityError(CompatibilityError):
    """The selected executor cannot accept a decision produced by the graph."""


@dataclass(frozen=True)
class ResolvedRunSpec:
    contract_version: str
    run_id: str
    execution_profile: str
    components: Mapping[str, str]
    episodes: int
    seed: int
    record: bool

    def __post_init__(self) -> None:
        if not isinstance(self.components, Mapping):
            raise ValueError("components must be a mapping")
        if any(
            not isinstance(name, str)
            or not name
            or not isinstance(plugin_id, str)
            or not plugin_id
            for name, plugin_id in self.components.items()
        ):
            raise ValueError("components must map non-empty strings to plugin ids")
        object.__setattr__(self, "components", MappingProxyType(dict(self.components)))

    def to_json(self) -> str:
        return json.dumps(
            {
                "components": dict(self.components),
                "contract_version": self.contract_version,
                "episodes": self.episodes,
                "execution_profile": self.execution_profile,
                "record": self.record,
                "run_id": self.run_id,
                "seed": self.seed,
            },
            sort_keys=True,
            separators=(",", ":"),
        )


def resolve_run(request: dict, catalog: Catalog) -> ResolvedRunSpec:
    """Resolve a request's IDs into a compatibility-checked, immutable RunSpec."""
    if not isinstance(request, dict):
        raise ValueError("run request must be a mapping")

    environment_id = _request_plugin_id(request, "environment_id")
    pipeline_id = _request_plugin_id(request, "pipeline_id")
    environment = _manifest(catalog, environment_id, "simulator")
    pipeline = _manifest(catalog, pipeline_id, "pipeline")

    runtime = pipeline.runtime
    execution_profile = _runtime_string(runtime, "execution_profile", pipeline_id)
    _manifest(catalog, execution_profile, "execution_profile")
    components = _runtime_components(runtime, pipeline_id)
    if "environment" not in components:
        raise ValueError(f"pipeline {pipeline_id} has no environment component")
    components["environment"] = environment.plugin_id

    _validate_capabilities(components, environment.plugin_id, catalog)
    _validate_decisions(components, catalog)

    return ResolvedRunSpec(
        contract_version=CONTRACT_VERSION,
        run_id=f"{datetime.now().strftime('%Y%m%d_%H%M%S')}_{token_hex(4)}",
        execution_profile=execution_profile,
        components=MappingProxyType(components),
        episodes=_positive_int(request.get("episodes", 1), "episodes"),
        seed=_integer(request.get("seed", 0), "seed"),
        record=_boolean(request.get("record", False), "record"),
    )


def _request_plugin_id(request: Mapping[str, Any], field: str) -> str:
    value = request.get(field)
    if not isinstance(value, str) or not value:
        raise ValueError(f"{field} must be a non-empty string")
    return value


def _manifest(catalog: Catalog, plugin_id: str, expected_kind: str):
    try:
        manifest = catalog.get(plugin_id)
    except KeyError as error:
        raise ValueError(f"unknown plugin id: {plugin_id}") from error
    if manifest.kind != expected_kind:
        raise ValueError(f"plugin {plugin_id} must be a {expected_kind}")
    return manifest


def _runtime_string(runtime: Mapping[str, Any], field: str, pipeline_id: str) -> str:
    value = runtime.get(field)
    if not isinstance(value, str) or not value:
        raise ValueError(f"pipeline {pipeline_id} runtime.{field} must be a non-empty string")
    return value


def _runtime_components(runtime: Mapping[str, Any], pipeline_id: str) -> dict[str, str]:
    value = runtime.get("components")
    if not isinstance(value, Mapping) or not value:
        raise ValueError(f"pipeline {pipeline_id} runtime.components must be a non-empty mapping")
    if any(not isinstance(name, str) or not isinstance(plugin_id, str) or not plugin_id for name, plugin_id in value.items()):
        raise ValueError(f"pipeline {pipeline_id} runtime.components must map strings to plugin ids")
    return dict(value)


def _validate_capabilities(components: Mapping[str, str], provider_id: str, catalog: Catalog) -> None:
    provider = catalog.get(provider_id)
    for component_id in components.values():
        if component_id == provider_id:
            continue
        component = catalog.get(component_id)
        missing = sorted(component.requires.difference(provider.provides))
        if missing:
            raise CapabilityError(
                {
                    "error": "incompatible_capabilities",
                    "component_id": component.plugin_id,
                    "provider_id": provider.plugin_id,
                    "missing": missing,
                }
            )


def _validate_decisions(components: Mapping[str, str], catalog: Catalog) -> None:
    producer_id = components.get("agent", components.get("decision"))
    executor_id = components.get("decision_executor")
    if producer_id is None or executor_id is None:
        return
    producer = catalog.get(producer_id)
    executor = catalog.get(executor_id)
    unsupported = sorted(producer.produces_decisions.difference(executor.accepts_decisions))
    if unsupported:
        raise DecisionCompatibilityError(
            {
                "error": "incompatible_decisions",
                "producer_id": producer.plugin_id,
                "executor_id": executor.plugin_id,
                "unsupported": unsupported,
            }
        )


def _positive_int(value: Any, field: str) -> int:
    result = _integer(value, field)
    if result < 1:
        raise ValueError(f"{field} must be positive")
    return result


def _integer(value: Any, field: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{field} must be an integer")
    return value


def _boolean(value: Any, field: str) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f"{field} must be a boolean")
    return value
