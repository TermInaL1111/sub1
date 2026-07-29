# decoup/ 与前端的交互架构

## 核心问题

1. **Topic 名硬编码** `/habitat/*` — 换 Isaac/MuJoCo/Go2 不能用同一套 topic
2. **前端 `server.py`** (luxinav-interactive-web) 用 `HabitatEvalBridgeClient` 硬编码发布到 `/habitat/*`
3. **decoup 节点** 也用 `/habitat/*`

**解决方案**: 统一为模拟器无关的 topic 命名空间／`HabitatEvalBridgeClient` topic 名可配置化

```
                      通用输入 (sim → planner / 前端)
       Habitat ──→ [ros2_http_bridge] ──→ /apexnav/obs/rgb
       Isaac   ──→ [isaac_bridge]    ──→ /apexnav/obs/depth
       MuJoCo  ──→ [mujoco_bridge]   ──→ /apexnav/obs/odom
       Go2 实机 ──→ [real_adapter]   ──→ /apexnav/obs/camera_pose
                                          /apexnav/obs/state

                      通用输出 (planner → sim)
       policy_node ──→ /apexnav/ctrl/plan_action (Int32)
       C++ FSM     ──→ /apexnav/ctrl/cmd_vel     (Twist)

                      感知输出 (VLM → planner)
       VLM 服务 ──→ /apexnav/perception/clouds_with_scores
                ──→ /apexnav/perception/cosine_score
```

---

## 全架构：三层都存在，各有原因

```
═══ CONDA PYTHON 3.9 ═══════════════   ═══ SYSTEM PYTHON 3.12 ════════════
 (habitat-sim, torch, VLM)             (rclpy, ROS 2, C++ FSM)
                                       
                                        ROS 2 Topic Bus
                                        ├── /apexnav/obs/rgb
                                        ├── /apexnav/obs/depth
                                        ├── /apexnav/obs/odom
                                        ├── /apexnav/obs/camera_pose
                                        ├── /apexnav/obs/state
                                        ├── /apexnav/obs/progress
                                        ├── /apexnav/ctrl/plan_action
                                        ├── /apexnav/ctrl/cmd_vel
                                        ├── /apexnav/ctrl/state (ROS state)
                                        ├── /apexnav/perception/clouds_with_scores
                                        └── /apexnav/perception/cosine_score

 栖息在 conda 侧:                       栖息在系统 Python 侧:
 ┌───────────────────────────┐         ┌───────────────────────────────┐
 │                          │         │                               │
 │ habitat_driver ──HTTP───┐│         │ ros2_http_bridge (:18080)     │
 │ (驱动 env.step)         ││         │ pub → /apexnav/obs/*          │
 │                          ││  POST   │ sub ← /apexnav/ctrl/*         │
 │ VLM ──HTTP────┐          ││  obs    │                               │
 │ (DINO/SAM/    │          ▼▼         │ policy_node                  │
 │  BLIP2/YOLO)  │      ┌──────────┐   │ sub sensors → pub plan_action │
 │ :12181-12184  │      │ bridge   │   │                               │
 └───────────────┘      │ client   │   │ collector_node               │
                        └──────────┘   │ sub topics → JSONL            │
                                       │                               │
                                       │ coordinator_node              │
 ┌───────────────────────────┐         │ monitor → metrics             │
 │ 前端 luxinav-interactive- │         │                               │
 │ web                       │         │ C++ FSM (exploration_node)    │
 │                          │         │ sub sensors → pub plan_action │
 │ server.py                │         │                               │
 │                          │         └───────────────────────────────┘
 │ HTTP server :8088        │
 │ WS → 浏览器 RGB 帧        │
 │                          │
 │ 当前: 用 HabitatEval     │
 │ BridgeClient 硬编码       │
 │ topic 名                 │
 │                          │
 │ 改造: topic 名可配置     │
 │ 改为 /apexnav/obs/*     │
 └───────────────────────────┘
```

---

## 需要改的文件

### 1. `habitat2ros/ros2_http_bridge.py` — bridge 端 topic 名可配置

当前硬编码:
```python
# 当前
self.rgb_pub = self.create_publisher(Image, "/habitat/camera_rgb", ...)
self.action_sub = self.create_subscription(Int32, "/habitat/plan_action", ...)

# 改为 ROS2 parameter
self.declare_parameter("rgb_topic", "/apexnav/obs/rgb")
self.declare_parameter("action_topic", "/apexnav/ctrl/plan_action")
rgb_topic = self.get_parameter("rgb_topic").value
self.rgb_pub = self.create_publisher(Image, rgb_topic, ...)
```

### 2. `habitat2ros/ros2_bridge_client.py` — 客户端 topic 名可配置

当前 `HabitatEvalBridgeClient.__init__` 硬编码:
```python
self.state_pub = "/habitat/state"
self.itm_score_pub = "/blip2/cosine_score"
```

改为:
```python
def __init__(self, topic_prefix="/apexnav"):
    self.state_pub = f"{topic_prefix}/obs/state"
    self.itm_score_pub = f"{topic_prefix}/perception/cosine_score"
```

### 3. decoup 节点 — 改用 `/apexnav/` 前缀

- `policy_node.py`: sub `/apexnav/obs/*`, pub `/apexnav/ctrl/plan_action`
- `collector_node.py`: sub `/apexnav/obs/*` 和 `/apexnav/ctrl/*`
- `mock_env_node.py`: pub `/apexnav/obs/state`

### 4. 前端 `server.py` — topic 名可配

`HabitatEvalBridgeClient` 传 `topic_prefix="/apexnav"`。

---

## 完整 topic 映射

| 旧 topic | 新 topic | 类型 | 方向 |
|---------|---------|------|------|
| `/habitat/camera_rgb` | `/apexnav/obs/rgb` | `sensor_msgs/Image` | sim → planner/前端 |
| `/habitat/camera_depth` | `/apexnav/obs/depth` | `sensor_msgs/Image` | sim → map_ros |
| `/habitat/odom` | `/apexnav/obs/odom` | `nav_msgs/Odometry` | sim → planner |
| `/habitat/sensor_pose` | `/apexnav/obs/camera_pose` | `nav_msgs/Odometry` | sim → map_ros |
| `/habitat/state` | `/apexnav/obs/state` | `std_msgs/Int32` | sim → planner |
| `/habitat/progress` | `/apexnav/obs/progress` | `Int32MultiArray` | sim → 外部监控 |
| `/habitat/plan_action` | `/apexnav/ctrl/plan_action` | `std_msgs/Int32` | planner → sim |
| `/cmd_vel` | `/apexnav/ctrl/cmd_vel` | `geometry_msgs/Twist` | planner → sim |
| `/ros/state` | `/apexnav/ctrl/state` | `std_msgs/Int32` | planner → sim |
| `/detector/clouds_with_scores` | `/apexnav/perception/clouds_with_scores` | `MultipleMasksWithConfidence` | VLM → planner |
| `/blip2/cosine_score` | `/apexnav/perception/cosine_score` | `std_msgs/Float64` | VLM → planner |

---

## 数据流（新 topic 名）

```
habitat_driver (conda)    ros2_http_bridge    ROS Topic Bus           policy_node / frontend
═══════════════════════    ════════════════    ════════════════       ═══════════════════════

env.step(action) 完成
→ POST /publish_observations ──▶ pub /apexnav/obs/rgb ──────────────▶ frontend 渲染帧
                                 pub /apexnav/obs/depth
                                 pub /apexnav/obs/odom
                                 pub /apexnav/obs/camera_pose
→ POST /publish_object_clouds ──▶ pub /apexnav/perception/clouds_with_scores
→ POST /publish_float64       ──▶ pub /apexnav/perception/cosine_score

→ POST /publish_int32         ──▶ pub /apexnav/obs/state=ACTION_FINISH ──▶ policy_node
  (state=2)                                                               decide() → action
                                                                           pub /apexnav/ctrl/plan_action

← GET /state                     ◀── bridge sub /apexnav/ctrl/plan_action
  拿到 action=1                                                          缓存 action
→ env.step(1)
```

---

## 与 C++ FSM 的兼容

C++ `exploration_fsm.cpp` 内部订阅 `/odom_world`，通过 launch 文件 remap:

```python
# exploration.launch.py 中已有的 remap:
remappings=[
    ('/odom_world', '/habitat/odom'),            # 旧
    ('/map_ros/pose', '/habitat/sensor_pose'),   # 旧
    ('/map_ros/depth', '/habitat/camera_depth'), # 旧
]

# 改为:
remappings=[
    ('/odom_world', '/apexnav/obs/odom'),
    ('/map_ros/pose', '/apexnav/obs/camera_pose'),
    ('/map_ros/depth', '/apexnav/obs/depth'),
]
```

C++ FSM 还通过 `/habitat/plan_action` 发布动作（也在 launch 中 remap 为 `/apexnav/ctrl/plan_action`）。核心原则：**C++ FSM 不改，只改 launch 文件 remap**。

---

## 启动（新 topic 树）

```bash
# 终端 1: VLM
docker compose up vlm

# 终端 2: ros2_http_bridge（新 topic 名，通过参数）
docker compose run --rm shell bash -lc "
  source /opt/ros/jazzy/setup.bash
  source install/setup.bash
  /usr/bin/python3 -m habitat2ros.ros2_http_bridge --port 18080 \
    --ros-args -p topic_prefix:=/apexnav
"

# 终端 3: habitat driver (conda，新的 bridge client topic_prefix)
docker compose run --rm shell bash -lc "
  conda activate apexnav
  cd /workspace/ApexNav
  python -m sim_bridge.habitat_wrapper --config-name habitat_eval_hm3dv2
"

# 终端 4: decoup policy node（sim-agnostic topic）
PYTHONPATH=/path/to/decoup \
/usr/bin/python3 -m sim_bridge.policy_node --policy random_explorer \
  --ros-args -p topic_prefix:=/apexnav

# 终端 5: 前端（topic 可配）
cd luxinav-interactive-web-20260723
python3 server.py --port 8088 --topic-prefix /apexnav
# 浏览器: http://127.0.0.1:8088/
```

**关键**：所有 ROS 节点用同一个 `--topic-prefix`，换模拟器不需要改代码。
