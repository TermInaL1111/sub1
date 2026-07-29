"""Validated, immutable representations of LuxiNav plugin manifests."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Any, Mapping


PLUGIN_SCHEMA_VERSION = "luxinav.plugin.v1"
PLUGIN_KINDS = frozenset(
    {
        "adapter",
        "agent",
        "execution_profile",
        "pipeline",
        "simulator",
        "tool_provider",
    }
)
_REQUIRED_KEYS = frozenset(
    {
        "schema_version",
        "kind",
        "id",
        "label",
        "provides",
        "requires",
        "accepts_decisions",
        "produces_decisions",
        "runtime",
    }
)


class ManifestValidationError(ValueError):
    """A manifest is invalid, with its source path retained in the message."""

    def __init__(self, source: Path, detail: str) -> None:
        self.source = source
        self.detail = detail
        super().__init__(f"{source}: {detail}")


@dataclass(frozen=True)
class PluginManifest:
    schema_version: str
    kind: str
    plugin_id: str
    label: str
    provides: frozenset[str]
    requires: frozenset[str]
    accepts_decisions: frozenset[str]
    produces_decisions: frozenset[str]
    runtime: Mapping[str, Any]


def parse_manifest(source: Path, value: Any) -> PluginManifest:
    """Validate an on-disk manifest and return an immutable value object."""
    if not isinstance(value, dict):
        raise ManifestValidationError(source, "manifest must be a mapping")

    missing = sorted(_REQUIRED_KEYS.difference(value))
    if missing:
        raise ManifestValidationError(source, f"missing required keys: {', '.join(missing)}")

    unexpected = sorted(set(value).difference(_REQUIRED_KEYS))
    if unexpected:
        raise ManifestValidationError(source, f"unknown keys: {', '.join(unexpected)}")

    schema_version = _string(source, "schema_version", value["schema_version"])
    if schema_version != PLUGIN_SCHEMA_VERSION:
        raise ManifestValidationError(source, f"unsupported schema version: {schema_version}")

    kind = _string(source, "kind", value["kind"])
    if kind not in PLUGIN_KINDS:
        raise ManifestValidationError(source, f"invalid plugin kind: {kind}")

    return PluginManifest(
        schema_version=schema_version,
        kind=kind,
        plugin_id=_string(source, "id", value["id"]),
        label=_string(source, "label", value["label"]),
        provides=_string_set(source, "provides", value["provides"]),
        requires=_string_set(source, "requires", value["requires"]),
        accepts_decisions=_string_set(source, "accepts_decisions", value["accepts_decisions"]),
        produces_decisions=_string_set(source, "produces_decisions", value["produces_decisions"]),
        runtime=_freeze_runtime(source, value["runtime"]),
    )


def _string(source: Path, field: str, value: Any) -> str:
    if not isinstance(value, str) or not value:
        raise ManifestValidationError(source, f"{field} must be a non-empty string")
    return value


def _string_set(source: Path, field: str, value: Any) -> frozenset[str]:
    if not isinstance(value, list) or any(not isinstance(item, str) or not item for item in value):
        raise ManifestValidationError(source, f"{field} must be a list of non-empty strings")
    return frozenset(value)


def _freeze_runtime(source: Path, value: Any) -> Mapping[str, Any]:
    if not isinstance(value, dict):
        raise ManifestValidationError(source, "runtime must be a mapping")
    return MappingProxyType(
        {key: _freeze_runtime_value(source, key, item) for key, item in value.items()}
    )


def _freeze_runtime_value(source: Path, key: Any, value: Any) -> Any:
    if not isinstance(key, str):
        raise ManifestValidationError(source, "runtime keys must be strings")
    if value is None or isinstance(value, (bool, float, int, str)):
        return value
    if isinstance(value, dict):
        return MappingProxyType(
            {
                child_key: _freeze_runtime_value(source, child_key, child_value)
                for child_key, child_value in value.items()
            }
        )
    raise ManifestValidationError(source, "runtime values must be immutable JSON scalars or mappings")
