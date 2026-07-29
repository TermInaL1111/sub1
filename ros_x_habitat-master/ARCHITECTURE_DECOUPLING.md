# ros_x_habitat 解耦架构分析

## 概述

`ros_x_habitat-master` 实现了 **Evaluator（评估器）/ ROS（中间件）/ Habitat（模拟器）** 三层解耦架构。核心思路是通过**抽象类 + 接口合约**将各层分离，使得任意一层可以被替换而不影响其他层。

```
┌─────────────────────────────────────────────────────┐
│                   Evaluator 层                       │
│   Evaluator (抽象)                                   │
│     └── HabitatSimEvaluator (共享逻辑)                │
│           ├── HabitatEvaluator (无 ROS，直接调用)      │
│           └── HabitatROSEvaluator (有 ROS，跨进程通信)  │
├─────────────────────────────────────────────────────┤
│                   ROS 中间件层 (可选)                  │
│   HabitatEnvNode          HabitatAgentNode           │
│   (包装 Simulator)        (包装 RL Agent)             │
│        │                       │                     │
│   pub: sensor topics    sub: sensor topics           │
│   sub: action topic     pub: action topic            │
│        │                       │                     │
│   ROS Topics: /rgb, /depth, /pointgoal, /action      │
│   ROS Services: EvalEpisode, ResetAgent              │
├─────────────────────────────────────────────────────┤
│              Simulator / Env 层                       │
│   HabitatRLEnv (gym.Env 接口)                         │
│     ├── Env (Habitat 原生，teleport 模式)              │
│     └── PhysicsEnv (Bullet 物理，连续速度控制)          │
│           └── PhysicsNavigationTask                   │
│                 └── _set_agent_velocities()           │
│                       (离散动作 → 线速度/角速度)        │
└─────────────────────────────────────────────────────┘
```

---

## 第一层：Evaluator 继承体系

### 抽象基类

**[src/evaluators/evaluator.py](src/evaluators/evaluator.py)** — 最顶层抽象，只定义一个接口：

```python
class Evaluator:
    def evaluate(episode_id_last, scene_id_last, log_dir) -> Dict[str, Dict[str, float]]:
        raise NotImplementedError
```

### 共享逻辑层

**[src/evaluators/habitat_sim_evaluator.py](src/evaluators/habitat_sim_evaluator.py)** — `HabitatSimEvaluator(Evaluator)`：

- 加载 Habitat config（`get_config`）
- 提供 `overwrite_simulator_config()`（物理模拟时覆写 SIMULATOR 配置）
- 提供共享工具方法：`compute_avg_metrics()`, `extract_metrics()`, `compute_pairwise_diff_of_metrics()`
- 定义抽象方法供子类实现：`generate_videos()`, `generate_maps()`, `evaluate_and_get_maps()`

### 两个具体实现 — 核心解耦点

| | HabitatEvaluator | HabitatROSEvaluator |
|---|---|---|
| **文件** | [habitat_evaluator.py](src/evaluators/habitat_evaluator.py) | [habitat_ros_evaluator.py](src/evaluators/habitat_ros_evaluator.py) |
| **ROS 依赖** | 无 | `rospy` ServiceProxy |
| **Env 创建** | 直接实例化 `HabitatEvalRLEnv` | 以子进程启动 `habitat_env_node.py` |
| **Agent 创建** | 直接实例化 `PPOAgent` | 以子进程启动 `habitat_agent_node.py` |
| **通信方式** | Python 直接调用 `env.step(action)` | ROS Service: `EvalEpisode`, `ResetAgent`, `GetAgentTime` |
| **适用场景** | 本地单机评测 | 分布式评测、跨语言 agent、Sim2Real 迁移 |

**关键设计**：两种 evaluator 共享完全相同的 `HabitatSimEvaluator` 基类，上层调用 `evaluate()` 的接口完全一致。是否使用 ROS 对调用者透明。

---

## 第二层：ROS 中间件 — Env/Agent 分离

ROS 模式下，模拟器和 agent 被分离为**两个独立的 ROS Node**，通过 **topic（数据流）+ service（控制流）** 通信。

### HabitatEnvNode（模拟器端）

**[src/nodes/habitat_env_node.py](src/nodes/habitat_env_node.py)**

```
对外接口：
  Publisher:
    /rgb          — sensor_msgs/Image (RGB 观测)
    /depth        — DepthImage 或 Image (深度观测)
    /pointgoal_with_gps_compass — PointGoalWithGPSCompass (导航目标)
  Subscriber:
    /action       — std_msgs/Int16 (离散动作 0-5)
    或 /cmd_vel   — geometry_msgs/Twist (连续速度，use_continuous_agent=True)
  Service:
    EvalEpisode   — 评估一个 episode
    Roam          — 自由漫游模式
```

内部同时支持两种控制模式：

| 模式 | 订阅 Topic | 指令类型 | Env 调用 |
|------|-----------|---------|----------|
| 离散 (`use_continuous_agent=False`) | `/action` (Int16) | 0-5 离散动作 | `env.step(action)` → 内部 dispatch 到 `Env.step()` 或 `PhysicsEnv.step_physics()` |
| 连续 (`use_continuous_agent=True`) | `/cmd_vel` (Twist) | 线速度 + 角速度 | `env.set_agent_velocities()` + `env.step()` |

### HabitatAgentNode（Agent 端）

**[src/nodes/habitat_agent_node.py](src/nodes/habitat_agent_node.py)**

```
对外接口：
  Subscriber:
    /rgb, /depth, /pointgoal_with_gps_compass — 时间同步的传感器数据
  Publisher:
    /action  — std_msgs/Int16 (agent 输出的动作)
  Service:
    ResetAgent   — 重置 agent
    GetAgentTime — 查询 agent 推理时间
```

内部封装 `PPOAgent`（Habitat Baselines 的 RL 模型），通过 `TimeSynchronizer` 同步多个传感器 topic。

### 通信时序

```
Evaluator              EnvNode                 AgentNode
   │                      │                        │
   │──EvalEpisode(srv)───▶│                        │
   │                      │──/rgb, /depth, /ptgoal─▶│
   │                      │                        │──agent.act()→ action
   │                      │◀──/action──────────────│
   │                      │──env.step(action)       │
   │                      │   (loop until done)     │
   │◀─metrics─────────────│                        │
   │                      │                        │
   │──ResetAgent(srv)──────────────────────────────▶│
```

---

## 第三层：Simulator — 双模式引擎

### HabitatRLEnv — gym.Env 包装器

**[src/envs/habitat_rlenv.py](src/envs/habitat_rlenv.py)**

```python
class HabitatRLEnv(gym.Env):
    def __init__(self, config, enable_physics=False):
        if enable_physics:
            self._env = PhysicsEnv(config)   # ← Bullet 物理模式
        else:
            self._env = Env(config)           # ← Habitat 原生 teleport 模式
```

`step()` 方法自动 dispatch：

```python
def step(self, *args, **kwargs):
    if self.enable_physics:
        observations = self._env.step_physics(*args, **kwargs)  # 物理步进
    else:
        observations = self._env.step(*args, **kwargs)           # 原生步进
```

### PhysicsEnv — 物理模拟扩展

**[src/envs/physics_env.py](src/envs/physics_env.py)** — 继承 `habitat.Env`，增加：

- 加载 LoCoBot 刚体模型作为 agent 的物理化身
- `set_agent_velocities(linear_vel, angular_vel)` — 直接控制 agent 对象的速度

### PhysicsNavigationTask — 离散→连续转换核心

**[src/tasks/habitat_physics_task.py:149-194](src/tasks/habitat_physics_task.py#L149-L194)**

这里是**离散动作转换为线速度/角速度**的算法所在：

```python
def _set_agent_velocities(self, action, agent_vel_control, control_period):
    if isinstance(action, MoveForwardAction):
        agent_vel_control.linear_velocity  = [0, 0, -0.25 / control_period]  # local -Z
        agent_vel_control.angular_velocity = [0, 0, 0]
    elif isinstance(action, TurnLeftAction):
        agent_vel_control.linear_velocity  = [0, 0, 0]
        agent_vel_control.angular_velocity = [0, deg2rad(10)/control_period, 0]  # local +Y
    elif isinstance(action, TurnRightAction):
        agent_vel_control.linear_velocity  = [0, 0, 0]
        agent_vel_control.angular_velocity = [0, deg2rad(-10)/control_period, 0]
```

**转换参数**：

| 离散动作 | 速度计算 | 数值 (control_period=1.0s) |
|---------|---------|--------------------------|
| MOVE_FORWARD | `[0, 0, -0.25/T]` | v = 0.25 m/s (local -Z) |
| TURN_LEFT | `[0, deg2rad(10)/T, 0]` | ω = 10°/s (local +Y) |
| TURN_RIGHT | `[0, deg2rad(-10)/T, 0]` | ω = -10°/s |
| STOP | `[0, 0, 0]` | 停止 |

**步进流程**（`step_physics()` 第 56-141 行）：
1. 设置速度控制 (`_set_agent_velocities`)
2. 计算总步数: `total_steps = control_period / time_step`
3. 逐帧调用 `self._sim.step_physics(agent_object, time_step)` （每帧 dt = 1/60s）
4. 碰撞检测后提前退出（可选）

### PhysicsSimulator 底层

**[src/sims/physics_simulator.py](src/sims/physics_simulator.py)** — 继承 `habitat_sim.Simulator`，重写 `step_physics()`：
- 调用 `super().step_world(dt)` 推进 Bullet 物理
- 执行碰撞检测 (`agent_object.contact_test()`)
- 返回传感器观测值

**[src/sims/habitat_physics_simulator.py](src/sims/habitat_physics_simulator.py)** — 继承 `PhysicsSimulator` + `habitat.Simulator`，向 Habitat Lab 注册为 `"Sim-Phys"` 模拟器。

---

## 第四层：跨模拟器桥接（Bonus）

项目还包含将 Habitat agent 与 Gazebo/其他模拟器桥接的节点，进一步展示解耦的能力：

### HabitatAgentToGazebo

**[src/nodes/habitat_agent_to_gazebo.py](src/nodes/habitat_agent_to_gazebo.py)**

将 **Habitat agent 的离散动作** 转换为 **Gazebo 的 `/cmd_vel` (Twist)**：

```python
if action_id == MOVE_FORWARD:
    linear_vel_local = [-0.25 / control_period, 0, 0]
    # 查询当前姿态，转换到世界坐标系
    linear_vel_world = quaternion_rotate_vector(rotation, linear_vel_local)
elif action_id == TURN_LEFT:
    vel_msg.angular.z = deg2rad(10.0)    # 10°/s
elif action_id == TURN_RIGHT:
    vel_msg.angular.z = deg2rad(-10.0)
```

动作完成后等待 `control_period` 秒，再发送 STOP。

### GazeboToHabitatAgent

**[src/nodes/gazebo_to_habitat_agent.py](src/nodes/gazebo_to_habitat_agent.py)**

反向桥接：订阅 Gazebo 的 RGB/Depth/Odom topic，转换为 Habitat agent 能理解的 sensor topic (`/rgb`, `/depth`, `/pointgoal_with_gps_compass`)。同时提供 `GetAgentPose` service 供 `HabitatAgentToGazebo` 查询姿态。

```
Gazebo ──RGB/Depth/Odom──▶ GazeboToHabitatAgent ──sensor topics──▶ HabitatAgentNode
                                                                        │
                                                                   /action (Int16)
                                                                        │
                                                                        ▼
Gazebo ◀──/cmd_vel (Twist)── HabitatAgentToGazebo ◀─────────────────────┘
```

---

## 解耦总结

| 替换目标 | 如何实现 | 修改范围 |
|---------|---------|---------|
| **不用 ROS** (直接本地评测) | 使用 `HabitatEvaluator` | 无需改动 Env/Agent 代码 |
| **用 ROS 分布式评测** | 使用 `HabitatROSEvaluator` | Env/Agent 各自作为独立 Node，不改核心逻辑 |
| **切换物理模式** | `enable_physics=True/False` | `HabitatRLEnv` 自动 dispatch 到 `Env` 或 `PhysicsEnv` |
| **离散 Agent → 连续速度控制** | `PhysicsNavigationTask._set_agent_velocities()` | 仅 Task 层，Env/Sim 层无感知 |
| **换模拟器 (Habitat → Gazebo)** | 使用 `GazeboToHabitatAgent` + `HabitatAgentToGazebo` 桥接 | Agent 代码完全不用改 |
| **换 Agent 模型** | 实现 `HabitatAgentNode` 的替代版本，遵守相同的 topic 合约 | Env Node 完全不用改 |
| **换传感器配置** | 修改 YAML config | 代码不用改 |

### 核心设计模式

1. **Template Method 模式** — `Evaluator.evaluate()` 定义骨架，子类实现具体步骤
2. **Strategy 模式** — `HabitatRLEnv` 根据 `enable_physics` 选择 `Env` 或 `PhysicsEnv`
3. **Publish/Subscribe 解耦** — ROS topic 作为统一的传感器/动作合约，Env 和 Agent 彼此不知道对方的实现细节
4. **Service 控制面** — ROS Service 处理生命周期（reset, shutdown），与数据流（topic）分离
