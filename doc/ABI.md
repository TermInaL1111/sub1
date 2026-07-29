# Solving the ABI Problem: Conda Python 3.9 vs ROS 2 System Python 3.12

## The Problem

ApexNav runs on a single Docker image, but **two incompatible Python stacks**:

| Component | Python | Key Dependency | Why |
|-----------|--------|---------------|-----|
| **Habitat-Sim** | 3.9 (conda `apexnav`) | `habitat-sim` (C++/PyBind) | Only prebuilt wheels for 3.9 |
| **ROS 2 Jazzy** | 3.12 (system) | `rclpy` (C extension) | Jazzy ships with system 3.12 |

`rclpy` cannot be imported in conda 3.9. `habitat-sim` cannot be imported in system 3.12. Any code that touches **both** will crash at import time.

## Three Patterns Used in This Project

### Pattern 1: HTTP Bridge (current `habitat_evaluation.py`)

```
┌─────────────────────────┐         HTTP (localhost)        ┌──────────────────────────┐
│  Conda Python 3.9       │  POST /publish_observations     │  System Python 3.12      │
│  (habitat, torch, vlm)  │  GET  /state?consume_action=1   │  (rclpy, ROS 2 topics)   │
│                          │ ──────────────────────────────▶ │                           │
│  HabitatEvalBridgeClient │                                 │  HabitatRosBridge (Node)  │
│  habitat_evaluation.py   │ ◀────────────────────────────── │  ros2_http_bridge.py      │
│  habitat_wrapper.py      │                                 │                           │
└─────────────────────────┘                                 └──────────────────────────┘
                                                                      │
                                                            ROS 2 pub/sub
                                                                      │
                                                            ┌─────────▼──────────┐
                                                            │  C++ Planner (FSM) │
                                                            │  exploration_node  │
                                                            └────────────────────┘
```

**How it works:**
1. `HabitatEvalBridgeClient` (conda 3.9) is a pure Python HTTP client — zero ROS imports.
2. `ros2_http_bridge.py` (system 3.12) is a standalone ROS 2 node launched as a subprocess.
3. Observations (numpy arrays) are base64-encoded NPZ blobs sent via HTTP POST.
4. Actions are polled via HTTP GET `/state?consume_action=1`.

**When to use:** You need the C++ `exploration_fsm` to receive sensor data and publish actions on ROS topics. The planner MUST be a ROS 2 node.

**Cost:** ~1-2ms serialization overhead per step. Subprocess lifecycle management (`ensure_bridge()`, health polling, restart).

---

### Pattern 2: Direct No-ROS (our `decoup/sim_bridge/` architecture)

```
┌──────────────────────────────────────────────┐
│  Conda Python 3.9                            │
│                                              │
│  HabitatSimAdapter ──▶ habitat.Env.step()    │
│       │                                      │
│  EpisodeRunner ──▶ Policy.predict()          │
│       │                │                     │
│       │         UnifiedAction                │
│       │                │                     │
│       └────────────────┘                     │
│                                              │
│  ZERO rclpy imports                          │
│  ZERO HTTP bridge                            │
│  ZERO subprocess                             │
└──────────────────────────────────────────────┘
```

**How it works:**
The `decoup/sim_bridge/` architecture **avoids the ABI problem entirely** by:

1. `HabitatSimAdapter` directly wraps `habitat.Env` — no ROS, no bridge.
2. `PolicyBase.predict()` returns `UnifiedAction` — a pure Python dataclass.
3. `EpisodeRunner` is the thinnest possible loop: `reset → predict → step`.
4. **No file in `decoup/sim_bridge/` imports `rclpy`.** The only optional ROS import is `geometry_msgs.msg.Twist` inside `UnifiedAction.to_twist()`, which is lazy-loaded only when called.

**When to use:** You are comparing navigation policies (random, MobileVLA-R1, any Python model) against Habitat episodes. You don't need the C++ FSM planner. This is the default for benchmarking and data collection.

**Why this works for most use cases:**
- Random explorer, heuristic, VLA models — all produce `UnifiedAction` directly.
- Simulator adapters consume `UnifiedAction` directly.
- No ROS topic contract needed between policy and simulator.

---

### Pattern 3: Hybrid with Subprocess Bridge (when C++ FSM is needed)

```
┌─────────────────────────┐         HTTP          ┌──────────────────────────┐
│  Conda Python 3.9       │                        │  System Python 3.12      │
│  decoup/sim_bridge/     │                        │  ros2_http_bridge.py     │
│                         │                        │                          │
│  HabitatSimAdapter      │──POST obs──────────────▶  publishes /habitat/*    │
│       │                 │                        │       │                  │
│  ApexNavPolicy          │◀─GET action────────────  subscribes /plan_action  │
│       │                 │                        │                          │
│  (polls bridge,         │                        └──────────────────────────┘
│   returns UnifiedAction)│                                   │
└─────────────────────────┘                         ROS 2 pub/sub
                                                              │
                                                    ┌─────────▼──────────┐
                                                    │  C++ Planner (FSM) │
                                                    └────────────────────┘
```

**How it works:**
1. `ApexNavPolicy` (conda 3.9) implements `PolicyBase.predict()` by polling `HabitatEvalBridgeClient` (HTTP).
2. The bridge (system 3.12, separate process) relays observations to ROS topics and actions back.
3. The C++ FSM receives ROS topics and publishes `/habitat/plan_action`.
4. `ApexNavPolicy` converts the action ID → `UnifiedAction` and returns it to `EpisodeRunner`.

**When to use:** You want to compare ApexNav's C++ FSM against other policies using the **same episode runner and metrics**. The benchmark matrix (`./run.sh benchmark`) does exactly this.

---

## Which Pattern Does What?

| Scenario | Pattern | ABI Solution |
|----------|---------|-------------|
| `./run.sh mock` | 2 (Direct) | No ROS needed at all |
| `./run.sh eval --policy random_explorer` | 2 (Direct) | `HabitatSimAdapter` + `RandomExplorer` both conda 3.9 |
| `./run.sh eval --policy mobilevla` | 2 (Direct) | MobileVLA inference runs in conda 3.9, no ROS |
| `./run.sh eval --policy apexnav` | 3 (Hybrid) | `ApexNavPolicy` talks to bridge subprocess |
| `./run.sh benchmark --policies random_explorer apexnav` | 2 + 3 mixed | Both patterns in same benchmark, same metrics |
| Original `habitat_evaluation.py` | 1 (HTTP Bridge) | Monolithic HTTP bridge approach |

---

## Key Insight

The decoupled architecture doesn't **solve** the ABI problem — it **sidesteps** it for the common case (policies that run in Python). The ABI split only appears when you need the C++ planner (Pattern 3), and even then, only the `ApexNavPolicy` adapter touches the bridge — the rest of the pipeline (`HabitatSimAdapter`, `EpisodeRunner`, metrics) is ABI-agnostic.
