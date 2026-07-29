from pathlib import Path

import pytest

from luxinav_runtime.catalog import Catalog
from luxinav_runtime.schema import ManifestValidationError


PLUGIN_ROOT = Path(__file__).parents[2] / "plugins"


@pytest.fixture
def catalog():
    return Catalog.load(PLUGIN_ROOT)


def test_public_payload_exposes_catalog_driven_environment_and_pipeline_choices(catalog):
    assert catalog.public_payload() == {
        "environments": [{"id": "mock", "label": "Deterministic Mock"}],
        "pipelines": [{"id": "mock_random", "label": "Mock Random Explorer"}],
    }


def test_manifest_validation_reports_source_path_for_missing_required_key(tmp_path):
    manifest = tmp_path / "simulators" / "missing-runtime.yaml"
    manifest.parent.mkdir()
    manifest.write_text(
        """\
schema_version: luxinav.plugin.v1
kind: simulator
id: missing_runtime
label: Missing runtime
provides: []
requires: []
accepts_decisions: []
produces_decisions: []
""",
        encoding="utf-8",
    )

    with pytest.raises(ManifestValidationError, match="runtime") as error:
        Catalog.load(tmp_path)

    assert str(manifest) in str(error.value)


def test_catalog_rejects_duplicate_plugin_ids_with_their_source_path(tmp_path):
    for kind in ("simulators", "agents"):
        directory = tmp_path / kind
        directory.mkdir()
        (directory / "duplicate.yaml").write_text(
            f"""\
schema_version: luxinav.plugin.v1
kind: {kind[:-1] if kind == "simulators" else "agent"}
id: duplicate
label: Duplicate
provides: []
requires: []
accepts_decisions: []
produces_decisions: []
runtime:
  service: duplicate-worker
""",
            encoding="utf-8",
        )

    with pytest.raises(ManifestValidationError, match="duplicate") as error:
        Catalog.load(tmp_path)

    assert "simulators/duplicate.yaml" in str(error.value)


@pytest.mark.parametrize(
    ("field", "value", "expected"),
    [
        ("schema_version", "luxinav.plugin.v2", "unsupported schema version"),
        ("kind", "unknown", "invalid plugin kind"),
    ],
)
def test_manifest_validation_rejects_unsupported_schema_or_plugin_kind(tmp_path, field, value, expected):
    manifest = tmp_path / "simulators" / "invalid.yaml"
    manifest.parent.mkdir()
    manifest.write_text(
        f"""\
schema_version: luxinav.plugin.v1
kind: simulator
id: invalid
label: Invalid
provides: []
requires: []
accepts_decisions: []
produces_decisions: []
runtime: {{}}
""".replace(f"{field}: " + ("luxinav.plugin.v1" if field == "schema_version" else "simulator"), f"{field}: {value}"),
        encoding="utf-8",
    )

    with pytest.raises(ManifestValidationError, match=expected) as error:
        Catalog.load(tmp_path)

    assert str(manifest) in str(error.value)


def test_manifest_runtime_is_deeply_immutable_after_loading(tmp_path):
    directory = tmp_path / "simulators"
    directory.mkdir()
    (directory / "immutable.yaml").write_text(
        """\
schema_version: luxinav.plugin.v1
kind: simulator
id: immutable
label: Immutable Runtime
provides: []
requires: []
accepts_decisions: []
produces_decisions: []
runtime:
  service: immutable-worker
  settings:
    retries: 2
""",
        encoding="utf-8",
    )

    runtime = Catalog.load(tmp_path).get("immutable").runtime

    with pytest.raises(TypeError):
        runtime["service"] = "changed"
    with pytest.raises(TypeError):
        runtime["settings"]["retries"] = 3
