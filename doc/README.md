# Decoupled Evaluation Platform

## Quick Start

```bash
cd /home/eikom/ApexNav/ros-x-habitat/decoup

# Verify everything works
./run.sh test          # 17 unit tests, ~1s
./run.sh mock          # Mock pipeline, 3 episodes, ~1s
./run.sh eval 2        # Real Habitat, 2 episodes, ~5 min

# Full verification
./run.sh all           # test → mock → eval
```

---

## Architecture

```
                     run.sh / run_eval.py
                           │
          ┌────────────────┼────────────────┐
          ▼                ▼                ▼
   ┌────────────┐  ┌────────────┐  ┌────────────┐
   │ ApexNav     │  │ MobileVLA  │  │ Random     │   ← policy/
   │ Policy      │  │ R1 Policy  │  │ Explorer   │
   └─────┬───────┘  └─────┬──────┘  └─────┬──────┘
         └────────┬───────┴───────────────┘
                  │
                  ▼
          UnifiedAction (vx, vy, yaw_rate, pitch, stop)
                  │
    ┌─────────────┼─────────────┐
    ▼             ▼             ▼
 HabitatSim     MockSim       MuJoCo       ← sim adapters
  Adapter      (test only)   (skeleton)

Outputs:
  eval/metrics.py         → EpisodeResult, compute_averages()
  eval/data_collector.py  → JSONL training traces
  eval/benchmark.py       → Policy × Simulator matrix
```

### Key Files

| File | Lines | Role |
|------|-------|------|
| `policy/unified_action.py` | 174 | UnifiedAction + discrete ↔ continuous conversion |
| `policy/policy_base.py` | 31 | Abstract Policy interface |
| `policy/policy_registry.py` | 43 | `@register` / `get_policy()` factory |
| `policy/random_explorer.py` | 18 | Random walk baseline |
| `policy/apexnav_policy.py` | 43 | ApexNav FSM bridge client |
| `policy/mobilevla_policy.py` | 51 | MobileVLA-R1 skeleton |
| `sim_adapter_base.py` | 43 | Abstract SimulatorBase interface |
| `habitat_sim_adapter.py` | 175 | Habitat-Sim adapter (direct env.Env) |
| `mock_simulator.py` | 80 | In-memory simulator for testing |
| `mujoco_wrapper.py` | 39 | MuJoCo skeleton |
| `perception/vlm_registry.py` | 108 | Pluggable VLM pipeline |
| `eval/episode_runner.py` | 78 | Standard reset→act→step loop |
| `eval/metrics.py` | 41 | Per-episode metrics + aggregation |
| `eval/benchmark.py` | 172 | Policy × simulator matrix runner |
| `eval/data_collector.py` | 96 | JSONL training data recorder |
| `run_eval.py` | 227 | CLI entry point |
| `run.sh` | — | Shell wrapper (auto-detects Docker) |
| `test/test_decoupled_e2e.py` | 221 | 17 integration tests |

---

## Continuous Velocity Parameters

Based on `habitat_physics_task.py::_set_agent_velocities()` (control_period=1.0s):

| Discrete Action | Continuous Velocity | Source |
|:--|:--|:--|
| `STOP` (0) | `vx=0, yaw_rate=0, stop=True` | |
| `MOVE_FORWARD` (1) | `vx = 0.25 m/s` | `_C.SIMULATOR.FORWARD_STEP_SIZE` |
| `TURN_LEFT` (2) | `yaw_rate = +10°/s (0.1745 rad/s)` | `_C.SIMULATOR.TURN_ANGLE` |
| `TURN_RIGHT` (3) | `yaw_rate = -10°/s` | `_C.SIMULATOR.TURN_ANGLE` |
| `TURN_DOWN` (4) | `pitch = -30° (-π/6)` | Instant tilt, not velocity |
| `TURN_UP` (5) | `pitch = +30° (+π/6)` | Instant tilt, not velocity |

### Discrete ↔ Continuous Conversion

```python
from sim_bridge.policy.unified_action import UnifiedAction

# Discrete → continuous (physics_task velocities)
a = UnifiedAction.from_discrete(1)   # MOVE_FORWARD → vx=0.25
a = UnifiedAction.from_discrete(2)   # TURN_LEFT → yaw_rate=+0.1745 rad/s

# Continuous → discrete (threshold-based)
u = UnifiedAction(vx=0.25)           # u.to_discrete() → 1
u = UnifiedAction(yaw_rate=0.1745)   # u.to_discrete() → 2

# To ROS Twist
twist = u.to_twist()                 # geometry_msgs/Twist
```

---

## Verification Results

### Docker Container (17/17 pass)

```
test_context_manager ........................ ok
test_record_and_read ....................... ok
test_full_loop_mock_sim .................... ok   ← MockSim + Random + EpisodeRunner
test_full_loop_with_data_collector ......... ok   ← Full pipeline with JSONL trace
test_compute_averages ...................... ok
test_empty_results ......................... ok
test_from_discrete_all_six_actions ......... ok   ← 0-5 roundtrip
test_to_discrete_roundtrip ................. ok
test_continuous_to_discrete_thresholds ..... ok
test_step_duration ......................... ok
test_to_twist ............................. ok
test_policy_registry_loads ................. ok   ← 3 policies registered
test_all_module_imports .................... ok
```

### Habitat Real Eval (2 episodes)

```
HabitatSimAdapter ready: config=habitat_eval_hm3dv2, episodes=1000
  Episode 1/2: id=1 scene=4ok3usBNeis.basis.glb target=toilet success=0 steps=500
  Episode 2/2: id=2 scene=4ok3usBNeis.basis.glb target=tv      success=0 steps=500

Habitat eval — 2/2 episodes, policy=random_explorer, dataset=hm3dv2
  Success rate:  0.0%       ← expected for random policy
  Avg SPL:       0.000
  Avg steps:     500.0      ← runs full episode
  Avg dist2goal: 7.053
```

### Mock Pipeline (3 episodes)

```
Mock eval complete — 3 episodes, policy=random_explorer
  Success rate:  67%
  Avg SPL:       0.567
  Avg steps:     5.0
```

---

## How to Add

### Add a new Policy

```python
# policy/my_policy.py
from sim_bridge.policy.policy_base import PolicyBase
from sim_bridge.policy.policy_registry import register
from sim_bridge.policy.unified_action import UnifiedAction

@register("my_policy")
class MyPolicy(PolicyBase):
    def predict(self, observations):
        # Your logic here
        return UnifiedAction(vx=0.25)

# Then import in policy/__init__.py
from sim_bridge.policy.my_policy import MyPolicy
```

Use: `python -m sim_bridge.run_eval --policy my_policy --dataset hm3dv2`

### Add a new Simulator

```python
# my_sim_adapter.py
from sim_bridge.sim_adapter_base import SimulatorBase

class MySimAdapter(SimulatorBase):
    def reset(self): ...
    def step(self, action: UnifiedAction): ...
    def is_episode_over(self): ...
    def get_metrics(self): ...
```

---

## What's Next

| Item | Status |
|------|--------|
| MobileVLA-R1 real inference | Pending — model code from GitHub `AIGeeksGroup/MobileVLA-R1` |
| Isaac Sim integration | Skeleton ready — needs Isaac ROS2 bridge |
| MuJoCo integration | Skeleton ready — needs `mujoco` Python bindings |
| ApexNav C++ FSM policy | Adapter exists — needs ros2_http_bridge running |
| Benchmark comparison table | CLI ready — needs multi-policy configured |
| ROS 2 launch files | TODO after Python modules verified |
