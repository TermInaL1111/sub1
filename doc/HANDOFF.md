# decoup/ 接手文档

## 一分钟理解

`decoup/` 把 `habitat_evaluation.py`（557 行单体）拆成独立的 ROS 2 节点，让你能：

- **换策略**：`--policy random_explorer` / `apexnav` / `mobilevla`
- **换模拟器**：Habitat / Isaac / MuJoCo（同 ROS topic 合约）
- **换 VLM**：full（DINO+YOLO+SAM+BLIP2）/ passthrough
- **做 benchmark**：策略 × 模拟器矩阵对比
- **采训练数据**：JSONL 格式，按 step 记录

---

## 环境

| 你需要知道 | 值 |
|-----------|-----|
| **项目根** | `/home/eikom/ApexNav` |
| **decoup 根** | `/home/eikom/ApexNav/ros-x-habitat/decoup` |
| **Docker 镜像** | `apexnav:jazzy-cuda128` |
| **conda env** | `apexnav` (Python 3.9) |
| **系统 Python** | `/usr/bin/python3` (3.12, 有 rclpy) |
| **ROS 2** | Jazzy, `rmw_cyclonedds_cpp` |
| **Compose 文件** | `/home/eikom/ApexNav/compose.yaml` |
| **前端** | `/home/eikom/ApexNav/luxinav-interactive-web-20260723/server.py` (port 8088) |

---

## 文件地图

```
ros-x-habitat/decoup/
│
├── sim_bridge/                    ← 我们的代码（核心）
│   ├── policy/
│   │   ├── unified_action.py     ← 通用动作 (vx,vy,yaw_rate,pitch,stop)
│   │   └── impls.py              ← 策略实现 (Random/ApexNav/MobileVLA)
│   ├── policy_node.py            ← ROS 2 Node: sub 传感器, pub /habitat/plan_action
│   ├── coordinator_node.py       ← ROS 2 Node: 监控 episode 指标
│   ├── collector_node.py         ← ROS 2 Node: 写 JSONL 训练数据
│   ├── mock_env_node.py          ← ROS 2 Node: 模拟 Habitat 状态机（测试用）
│   ├── run_eval.py               ← 启动脚本，自动选 /usr/bin/python3 跑 ROS 节点
│   ├── eval/metrics.py           ← EpisodeResult + compute_averages()
│   ├── perception/vlm_registry.py← VLM 管道 (full/passthrough)
│   └── test/test_e2e.py          ← 17 个测试（含全链路 ROS 集成）
│
├── ros_x_habitat-master/         ← 参考架构（ROS 1，不要改）
│   ├── src/evaluators/           ← Evaluator 三层继承
│   ├── src/nodes/                ← EnvNode + AgentNode 分离
│   ├── src/tasks/habitat_physics_task.py ← 离散→连续速度转换（参数来源）
│   └── FOLDER_DECOUPLING.md      ← 每个目录的解耦说明
│
└── doc/                          ← 文档
    ├── HANDOFF.md                ← 你正在读的
    ├── README.md                 ← 架构 + 接入指南
    ├── ABI.md                    ← conda 3.9 vs 系统 3.12 的 HTTP bridge 方案
    ├── FRONTEND_INTERACTION.md   ← 与前端交互架构
    └── CODE_REVIEW.md            ← 启动失败 bug 分析
```

---

## 快速验证

```bash
# === 非 ROS 测试（宿主编译，秒级） ===
cd /home/eikom/ApexNav/ros-x-habitat/decoup
python3 -c "
from sim_bridge.policy.unified_action import UnifiedAction
from sim_bridge.policy.impls import get_policy_impl
from sim_bridge.eval.metrics import EpisodeResult, compute_averages
# 离散→连续→离散往返
a = UnifiedAction.from_discrete(2)
assert a.to_discrete() == 2, 'roundtrip failed'
print('All non-ROS OK')
"

# === ROS 2 全链路测试（Docker 内，60s） ===
sudo docker compose run --rm shell bash -lc "
export PYTHONPATH=/workspace/ApexNav/ros-x-habitat/decoup:\$PYTHONPATH
cd /workspace/ApexNav/ros-x-habitat/decoup
python -m unittest sim_bridge.test.test_e2e -v
"
# 预期: 17 passed (1 skipped if no ROS)
```

---

## run.sh 命令

```bash
cd /home/eikom/ApexNav/ros-x-habitat/decoup

# 宿主端（自动进 Docker）
./run.sh test       # 17 单元测试
./run.sh mock 5     # Mock 管道 5 集
./run.sh eval 10    # 真 Habitat 10 集
./run.sh benchmark  # 策略矩阵对比
./run.sh all        # 全量验证

# Docker 内直接跑（更快）
sudo docker compose run --rm shell bash -lc "
export PYTHONPATH=/workspace/ApexNav/ros-x-habitat/decoup:\$PYTHONPATH
cd /workspace/ApexNav/ros-x-habitat/decoup
python -m sim_bridge.run_eval --mock --policy random_explorer --episodes 3
"
```

---

## 核心概念

### UnifiedAction — 策略和模拟器之间的共同语言

```python
from sim_bridge.policy.unified_action import UnifiedAction

# 离算→连续 (速度参数来自 habitat_physics_task.py)
a = UnifiedAction.from_discrete(1)  # MOVE_FORWARD → vx=0.25 m/s
a = UnifiedAction.from_discrete(2)  # TURN_LEFT → yaw_rate=+10°/s

# 连续→离算 (阈值判定)
u = UnifiedAction(vx=0.25)          # u.to_discrete() → 1
u = UnifiedAction(yaw_rate=0.1745)  # u.to_discrete() → 2

# 转 ROS Twist
twist = u.to_twist()  # geometry_msgs/Twist
```

### ABI 分界 — HTTP bridge

```
═══ CONDA 3.9 ═══════════════════   ═══ SYSTEM 3.12 ═══════════════
 habitat-sim, torch, VLM              rclpy, ROS 2, C++ FSM
        │                                    │
        ├── HTTP POST obs ───────────────▶ ros2_http_bridge (:18080)
        │                                    │ pub /habitat/*
        │                                    │ sub /habitat/plan_action
        ├── HTTP GET /state ◀─────────────── ros2_http_bridge
        │   (拿 action)
```

### 策略节点 — 替换 C++ FSM

`policy_node.py` 和 C++ `exploration_node` **互斥**——都发 `/habitat/plan_action`：

```bash
# 用 Python 策略（不启动 C++ FSM）
/usr/bin/python3 -m sim_bridge.policy_node --policy random_explorer

# 用 C++ FSM（不启动 policy_node）
docker compose up algorithm

# 用 ApexNav bridge（policy_node 走 HTTP 调 C++ FSM）
/usr/bin/python3 -m sim_bridge.policy_node --policy apexnav
```

---

## 如何接入新策略

在 `sim_bridge/policy/impls.py` 加一个类：

```python
class MyPolicy(PolicyImpl):
    def decide(self, rgb, depth, odom, state) -> UnifiedAction:
        # 你的逻辑
        return UnifiedAction(vx=0.25)

# 末尾注册表加一行
_POLICY_REGISTRY["my_policy"] = MyPolicy
```

使用：

```bash
/usr/bin/python3 -m sim_bridge.policy_node --policy my_policy
```

---

## 如何接入新模拟器

1. 继承父级 `sim_bridge/sim_wrapper_base.py` 的 `SimWrapperBase`
2. 实现 5 个方法：`_reset()`, `_step(action_id)`, `_is_episode_over()`, `_get_metrics()`, `_get_current_obs()`
3. 发布到标准 ROS topics：`/habitat/camera_rgb`, `/habitat/odom`, `/habitat/state`
4. 订阅 `/habitat/plan_action`

MuJoCo skeleton 参考：`decoup/ros_x_habitat-master/src/nodes/habitat_env_node.py`

---

## ROS Topic 合约（完整）

| Topic | 类型 | 方向 | 用途 |
|-------|------|------|------|
| `/habitat/camera_rgb` | `Image` (rgb8) | sim → planner/前端 | RGB 观测 |
| `/habitat/camera_depth` | `Image` (32FC1) | sim → map | 深度图 |
| `/habitat/odom` | `Odometry` | sim → planner | 里程计 |
| `/habitat/sensor_pose` | `Odometry` | sim → map | 相机位姿 |
| `/habitat/state` | `Int32` | sim → planner | 状态机 (0=READY, 2=ACTION_FINISH, 3=EPISODE_FINISH) |
| `/habitat/plan_action` | `Int32` | planner → sim | 离散动作 0-5 |
| `/blip2/cosine_score` | `Float64` | VLM → planner | 语义分数 |
| `/detector/clouds_with_scores` | `MultipleMasksWithConfidence` | VLM → planner | 目标点云 |

**未来**：迁移到 `/apexnav/obs/*` `/apexnav/ctrl/*` `/apexnav/perception/*`（模拟器无关命名，见 `FRONTEND_INTERACTION.md`）。

---

## 前端启动（含 decoup）

```bash
# 终端 1: VLM
docker compose up vlm

# 终端 2: ros2_http_bridge
sudo docker compose run --rm shell bash -lc "
  source /opt/ros/jazzy/setup.bash && source install/setup.bash
  /usr/bin/python3 -m habitat2ros.ros2_http_bridge --port 18081
"

# 终端 3: decoup policy（替代 C++ FSM）
PYTHONPATH=/home/eikom/ApexNav/ros-x-habitat/decoup \
/usr/bin/python3 -m sim_bridge.policy_node --policy random_explorer

# 终端 4: decoup collector（可选）
PYTHONPATH=/home/eikom/ApexNav/ros-x-habitat/decoup \
/usr/bin/python3 -m sim_bridge.collector_node \
  --ros-args -p output_path:=/tmp/frontend_trace.jsonl

# 终端 5: Habitat driver（conda 侧）
sudo docker compose run --rm shell bash -lc "
  cd /workspace/ApexNav
  export PYTHONPATH=.:ros-x-habitat/decoup:\$PYTHONPATH
  python -m sim_bridge.habitat_env_node \
    --config-name habitat_eval_hm3dv2 --max-steps 500 \
    --bridge-url http://127.0.0.1:18080
"

# 终端 6: 前端
cd /home/eikom/ApexNav/luxinav-interactive-web-20260723
APEXNAV_ROOT=/home/eikom/ApexNav python3 server.py --host 0.0.0.0 --port 8088

# 浏览器: http://127.0.0.1:8088/
```

---

## 常见问题

### 1. 容器名出现 `LuxiNav` 而非 `apexnav`

根因：`server.py` 的 `compose_cmd` 没有强制 `-p` 项目名。CWD 目录名 `luxinav-interactive-web-20260723` 导致 Docker Compose 自动生成 `LuxiNav` 前缀。

修复（已在代码中）：`compose_cmd` 加 `-p apexnav -f /path/to/compose.yaml`。

### 2. ModuleNotFoundError: No module named 'sim_bridge'

- 在 **Docker 外**：PYTHONPATH 需包含 `ros-x-habitat/decoup`
- 在 **Docker 内**：`cd /workspace/ApexNav && PYTHONPATH=.:ros-x-habitat` 即可

### 3. rclpy 无法 import (ABI 错误)

`conda python 3.9` 无法 import rclpy。ROB 节点必须用 `/usr/bin/python3`（系统 3.12）。`run_eval.py` 自动选择正确的 Python。

---

## 关键文件索引（外部依赖）

| 文件 | 用途 |
|------|------|
| `/home/eikom/ApexNav/habitat_evaluation.py` | 要被解耦的旧单体 |
| `/home/eikom/ApexNav/habitat2ros/ros2_http_bridge.py` | HTTP↔ROS bridge (系统 3.12) |
| `/home/eikom/ApexNav/habitat2ros/ros2_bridge_client.py` | HTTP client (conda 3.9) |
| `/home/eikom/ApexNav/ros-x-habitat/sim_bridge/sim_wrapper_base.py` | SimWrapperBase(rclpy.Node) |
| `/home/eikom/ApexNav/ros-x-habitat/sim_bridge/habitat_wrapper.py` | Habitat wrapper (conda) |
| `/home/eikom/ApexNav/ros-x-habitat/sim_bridge/habitat_env_node.py` | Habitat env node (conda) |
| `/home/eikom/ApexNav/ros-x-habitat/sim_bridge/mock_wrapper.py` | Mock rclpy simulator |
| `/home/eikom/ApexNav/ros-x-habitat/sim_bridge/entrypoint.sh` | 选 conda vs system Python |
| `/home/eikom/ApexNav/ros-x-habitat/sim_bridge/action_constants.py` | CommonAction + ACTION 枚举 |
| `/home/eikom/ApexNav/src/planner/exploration_manager/src/exploration_fsm.cpp` | C++ FSM |
| `/home/eikom/ApexNav/compose.yaml` | Docker Compose 服务定义 |
| `/home/eikom/ApexNav/docs/model_adapter_api.md` | L0-L3 adapter 合约 |
| `/home/eikom/ApexNav/luxinav-interactive-web-20260723/server.py` | 前端 server |
