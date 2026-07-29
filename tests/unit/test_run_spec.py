import json
import re
from pathlib import Path

import pytest

from luxinav_runtime.catalog import Catalog
from luxinav_runtime.run_spec import (
    CapabilityError,
    DecisionCompatibilityError,
    ResolvedRunSpec,
    resolve_run,
)


PLUGIN_ROOT = Path(__file__).parents[2] / "plugins"


def write_manifest(root, relative_path, body):
    path = root / relative_path
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "schema_version: luxinav.plugin.v1\n"
        + "provides: []\n"
        + "requires: []\n"
        + "accepts_decisions: []\n"
        + "produces_decisions: []\n"
        + "runtime: {}\n"
        + body,
        encoding="utf-8",
    )


@pytest.fixture
def catalog():
    return Catalog.load(PLUGIN_ROOT)


def test_mock_random_pipeline_resolves_direct_agent_graph(catalog):
    spec = resolve_run(
        {
            "environment_id": "mock",
            "pipeline_id": "mock_random",
            "episodes": 3,
            "seed": 7,
            "record": True,
        },
        catalog,
    )

    assert spec.execution_profile == "direct_agent"
    assert spec.components == {
        "environment": "mock",
        "agent": "random_explorer",
        "decision_executor": "direct_control",
    }
    assert list(spec.components) == ["environment", "agent", "decision_executor"]
    assert spec.contract_version == "luxinav.v1"
    assert re.fullmatch(r"\d{8}_\d{6}_[0-9a-f]{8}", spec.run_id)


def test_resolved_run_spec_is_immutable_and_serializes_canonically(catalog):
    spec = resolve_run(
        {
            "environment_id": "mock",
            "pipeline_id": "mock_random",
            "episodes": 3,
            "seed": 7,
            "record": True,
        },
        catalog,
    )

    with pytest.raises(TypeError):
        spec.components["agent"] = "other-agent"

    payload = json.loads(spec.to_json())
    assert payload == {
        "components": {
            "agent": "random_explorer",
            "decision_executor": "direct_control",
            "environment": "mock",
        },
        "contract_version": "luxinav.v1",
        "episodes": 3,
        "execution_profile": "direct_agent",
        "record": True,
        "run_id": spec.run_id,
        "seed": 7,
    }
    assert spec.to_json() == json.dumps(payload, sort_keys=True, separators=(",", ":"))


def test_resolved_run_spec_does_not_retain_a_callers_mutable_component_mapping():
    source_components = {"environment": "mock", "agent": "random_explorer"}
    spec = ResolvedRunSpec(
        contract_version="luxinav.v1",
        run_id="20260729_120000_deadbeef",
        execution_profile="direct_agent",
        components=source_components,
        episodes=1,
        seed=7,
        record=False,
    )
    source_components["agent"] = "changed"

    assert spec.components["agent"] == "random_explorer"
    with pytest.raises(TypeError):
        spec.components["agent"] = "changed-again"


def test_resolution_reports_missing_capability(tmp_path):
    plugins = tmp_path / "plugins"
    write_manifest(
        plugins,
        "simulators/mock_without_rgb.yaml",
        """\
kind: simulator
id: mock_without_rgb
label: Mock without RGB
provides: [depth]
""",
    )
    write_manifest(
        plugins,
        "agents/rgb_agent.yaml",
        """\
kind: agent
id: rgb_agent
label: RGB Agent
requires: [rgb]
produces_decisions: [continuous_control]
""",
    )
    write_manifest(
        plugins,
        "adapters/direct_control.yaml",
        """\
kind: adapter
id: direct_control
label: Direct Control
accepts_decisions: [continuous_control]
""",
    )
    write_manifest(
        plugins,
        "execution_profiles/direct_agent.yaml",
        """\
kind: execution_profile
id: direct_agent
label: Direct Agent
""",
    )
    write_manifest(
        plugins,
        "pipelines/rgb_agent.yaml",
        """\
kind: pipeline
id: rgb_agent_pipeline
label: RGB Agent Pipeline
runtime:
  execution_profile: direct_agent
  components:
    environment: mock_without_rgb
    agent: rgb_agent
    decision_executor: direct_control
""",
    )

    with pytest.raises(CapabilityError, match="rgb") as error:
        resolve_run(
            {"environment_id": "mock_without_rgb", "pipeline_id": "rgb_agent_pipeline"},
            Catalog.load(plugins),
        )

    assert error.value.payload == {
        "error": "incompatible_capabilities",
        "component_id": "rgb_agent",
        "provider_id": "mock_without_rgb",
        "missing": ["rgb"],
    }


def test_resolution_rejects_a_decision_the_executor_does_not_accept(tmp_path):
    plugins = tmp_path / "plugins"
    manifests = {
        "simulators/mock.yaml": """\
kind: simulator
id: mock
label: Mock
provides: [pose]
""",
        "agents/discrete_agent.yaml": """\
kind: agent
id: discrete_agent
label: Discrete Agent
requires: [pose]
produces_decisions: [discrete_action]
""",
        "adapters/continuous_only.yaml": """\
kind: adapter
id: continuous_only
label: Continuous Only
accepts_decisions: [continuous_control]
""",
        "execution_profiles/direct.yaml": """\
kind: execution_profile
id: direct
label: Direct
""",
        "pipelines/bad_pipeline.yaml": """\
kind: pipeline
id: bad_pipeline
label: Bad Pipeline
runtime:
  execution_profile: direct
  components:
    environment: mock
    agent: discrete_agent
    decision_executor: continuous_only
""",
    }
    for relative_path, body in manifests.items():
        write_manifest(plugins, relative_path, body)

    with pytest.raises(DecisionCompatibilityError, match="discrete_action"):
        resolve_run(
            {"environment_id": "mock", "pipeline_id": "bad_pipeline"},
            Catalog.load(plugins),
        )
