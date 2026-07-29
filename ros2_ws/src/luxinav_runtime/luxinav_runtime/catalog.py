"""Deterministic loading and public presentation of LuxiNav plugin manifests."""

from __future__ import annotations

from pathlib import Path
from types import MappingProxyType
from typing import Mapping

import yaml

from .schema import ManifestValidationError, PluginManifest, parse_manifest


class Catalog:
    """An immutable plugin catalog loaded from a directory tree."""

    def __init__(self, manifests: Mapping[str, PluginManifest]) -> None:
        self._manifests = MappingProxyType(dict(sorted(manifests.items())))

    @classmethod
    def load(cls, root: Path) -> "Catalog":
        manifests: dict[str, PluginManifest] = {}
        for source in sorted(root.rglob("*.yaml")):
            manifest = _load_manifest(source)
            if manifest.plugin_id in manifests:
                raise ManifestValidationError(source, f"duplicate plugin id: {manifest.plugin_id}")
            manifests[manifest.plugin_id] = manifest
        return cls(manifests)

    def get(self, plugin_id: str) -> PluginManifest:
        return self._manifests[plugin_id]

    def by_kind(self, kind: str) -> tuple[PluginManifest, ...]:
        return tuple(manifest for manifest in self._manifests.values() if manifest.kind == kind)

    def public_payload(self) -> dict:
        return {
            "environments": [
                {"id": manifest.plugin_id, "label": manifest.label}
                for manifest in self.by_kind("simulator")
            ],
            "pipelines": [
                {"id": manifest.plugin_id, "label": manifest.label}
                for manifest in self.by_kind("pipeline")
            ],
        }


def _load_manifest(source: Path) -> PluginManifest:
    try:
        with source.open(encoding="utf-8") as stream:
            value = yaml.safe_load(stream)
    except yaml.YAMLError as error:
        raise ManifestValidationError(source, f"invalid YAML: {error}") from error
    return parse_manifest(source, value)
