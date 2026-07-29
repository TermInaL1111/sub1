# ros_x_habitat-master: 每个目录如何实现解耦

## 目录全景

```
ros_x_habitat-master/
├── src/
│   ├── evaluators/     ← 评估编排层 (最顶层)
│   ├── nodes/          ← ROS 中间件层 (Env ↔ Agent 桥接)
│   ├── envs/           ← Gym RL 环境层 (统一接口)
│   ├── sims/           ← 模拟器底层 (Habitat-Sim 物理引擎)
│   ├── tasks/          ← 任务定义层 (导航逻辑)
│   ├── constants/      ← 共享枚举
│   ├── measures/       ← 观测度量
│   ├── roamers/        ← 自由漫游
│   ├── scripts/        ← 评估/可视化脚本
│   ├── test/           ← 集成测试
│   └── utils/          ← 工具函数
├── configs/            ← YAML 配置文件
├── maps/               ← 导航地图
├── navigation_params/  ← ROS Navigation 参数
├── launch/             ← ROS Launch 文件
├── msg/                ← 自定义 ROS 消息
└── srv/                ← 自定义 ROS 服务
```

---

## 1. `src/evaluators/` — 评估编排层 (架构最顶层)

**解耦了什么:** 评估器与 ROS 中间件的分离。

| 文件 | 继承关系 | 职责 |
|------|---------|------|
| `evaluator.py` | 抽象基类 | 只定义 `evaluate()` 一个接口，不依赖任何具体实现 |
| `habitat_sim_evaluator.py` | `evaluator.py` → | Habitat 相关的共享逻辑 (config 加载、指标聚合)，但**不引入 ROS** |
| `habitat_evaluator.py` | → `habitat_sim_evaluator.py` | 无 ROS 模式：直接 new `HabitatEvalRLEnv` + `PPOAgent`，纯 Python 进程内调用 |
| `habitat_ros_evaluator.py` | → `habitat_sim_evaluator.py` | 有 ROS 模式：启动 env node + agent node 子进程，只通过 ROS Service 通信 |

**关键设计:**
```python
# 同一个基类，两个子类，上层调用完全一致
class Evaluator(ABC):
    def evaluate(self) -> dict: ...

# 无 ROS：直接调用
class HabitatEvaluator(HabitatSimEvaluator):
    def evaluate(self):
        self.env = HabitatEvalRLEnv(config)
        action = self.agent.act(obs)
        obs = self.env.step(action)

# 有 ROS：跨进程通信
class HabitatROSEvaluator(HabitatSimEvaluator):
    def evaluate(self):
        resp = self.eval_episode_service(episode_id, scene_id)  # ROS srv
        action = self.reset_agent_service(RESET, seed)          # ROS srv
```

**切换方式:** 选择哪个 Evaluator 子类，不影响上层调用者。

---

## 2. `src/nodes/` — ROS 中间件层

**解耦了什么:** 模拟器与 Agent 的物理分离。Env 和 Agent 各自是独立 ROS Node，彼此不知道对方实现。

| 文件 | 角色 | 发布 | 订阅 |
|------|------|------|------|
| `habitat_env_node.py` | 模拟器包装 | `/rgb`, `/depth`, `/pointgoal_with_gps_compass` | `/action` 或 `/cmd_vel` |
| `habitat_agent_node.py` | RL Agent 包装 | `/action` (Int16) | `/rgb`, `/depth`, `/pointgoal_with_gps_compass` |
| `gazebo_to_habitat_agent.py` | 桥接: Gazebo → Habitat | `/rgb`, `/depth`, `/pointgoal` | Gazebo 传感器 topics |
| `habitat_agent_to_gazebo.py` | 桥接: Habitat → Gazebo | `/cmd_vel` (Twist) | `/action` (Int16) |

**关键设计 — Env/Agent 通过标准 Topic 合约通信:**
```
HabitatEnvNode ──/rgb,/depth,/ptgoal──▶ HabitatAgentNode
                                        agent.act() → action
HabitatEnvNode ◀──/action────────────── HabitatAgentNode
                env.step(action)
```

换 Agent 不需要改 Env Node（只要新 Agent 发布 `/action` Int16），换 Env 不需要改 Agent Node（只要新 Env 发布 `/rgb` `/depth` `/ptgoal`）。

**Gazebo 桥接证明了这个解耦:**
```
Gazebo ──sensors──▶ GazeboToHabitatAgent ──/rgb,/depth──▶ HabitatAgentNode (不改!)
                                                                 │
                                                            /action
                                                                 │
Gazebo ◀──/cmd_vel── HabitatAgentToGazebo ◀──────────────────────┘ (不改!)
```

---

## 3. `src/envs/` — RL 环境抽象层

**解耦了什么:** 物理模式与 teleport 模式的切换。上层调用 `step()` 不需要知道底层是否有 Bullet 物理。

| 文件 | 继承 | 职责 |
|------|------|------|
| `habitat_rlenv.py` | `gym.Env` | 统一 `reset()/step()` 接口，根据 `enable_physics` flag 自动 dispatch |
| `habitat_eval_rlenv.py` | `habitat_rlenv.py` → | 评估专用：reward=0, done=episode_over, info=metrics |
| `physics_env.py` | `habitat.Env` → | 加载 LoCoBot 刚体模型，`set_agent_velocities()` 直接控制速度 |

**关键设计:**
```python
class HabitatRLEnv(gym.Env):
    def __init__(self, config, enable_physics=False):
        if enable_physics:
            self._env = PhysicsEnv(config)    # Bullet 物理
        else:
            self._env = Env(config)           # 原生 teleport

    def step(self, action):
        if self.enable_physics:
            return self._env.step_physics(action)   # 物理步进
        else:
            return self._env.step(action)            # 原生步进
```

切换物理开关: `enable_physics=True/False` — 一行代码。

---

## 4. `src/sims/` — 模拟器底层

**解耦了什么:** Habitat-Sim C++ 引擎与 Habitat-Lab Python 接口的适配。

| 文件 | 继承 | 职责 |
|------|------|------|
| `physics_simulator.py` | `habitat_sim.Simulator` → | 重写 `step_physics()`，逐帧步进 Bullet 物理，每帧做碰撞检测 |
| `habitat_physics_simulator.py` | `PhysicsSimulator` + `habitat.Simulator` → | 实现 habitat-lab 的 `Simulator` 抽象接口，注册为 `"Sim-Phys"` |

**关键设计 — 通过 Habitat Registry 实现可替换:**
```python
@registry.register_simulator(name="Sim-Phys")
class HabitatPhysicsSim(PhysicsSimulator, Simulator):
    def step_physics(self, agent_object, time_step):
        sim_obs = super().step_physics(agent_object, time_step)
        return self._sensor_suite.get_observations(sim_obs)
```

YAML 配置中选择 `Sim-Phys` 就是物理模式；选择默认 `Sim-v0` 就是 teleport 模式。**代码不用改。**

---

## 5. `src/tasks/` — 任务定义层

**解耦了什么:** 导航任务逻辑与模拟器物理引擎的分离。

| 文件 | 继承 | 职责 |
|------|------|------|
| `habitat_physics_task.py` | `EmbodiedTask` → | 定义物理导航任务 `"Nav-Phys"`，**最关键的方法是 `_set_agent_velocities()`** |

**离算动作 → 连续速度的核心转换在此:**
```python
class PhysicsNavigationTask:
    def _set_agent_velocities(self, action, agent_vel_control, control_period):
        if isinstance(action, MoveForwardAction):
            agent_vel_control.linear_velocity  = [0, 0, -0.25 / control_period]
        elif isinstance(action, TurnLeftAction):
            agent_vel_control.angular_velocity = [0, deg2rad(10)/control_period, 0]
        elif isinstance(action, TurnRightAction):
            agent_vel_control.angular_velocity = [0, deg2rad(-10)/control_period, 0]
```

这个转换是 `decoup/sim_bridge/policy/unified_action.py` 中 `from_discrete()` 速度值的直接来源。

---

## 6. `src/constants/` — 共享枚举

**解耦了什么:** 跨模块的常量统一（避免硬编码）。

```python
class AgentResetCommands:  # RESET=0, SHUTDOWN=1
class EvalEpisodeSpecialIDs:  # REQUEST_NEXT="-1", REQUEST_SHUTDOWN="-2"
class NumericalMetrics:  # DISTANCE_TO_GOAL, SUCCESS, SPL, etc.
class ServiceNames:      # EVAL_EPISODE, RESET_AGENT, GET_AGENT_TIME, etc.
```

---

## 7. `src/measures/` — 观测度量

`top_down_map_for_roam.py` — 自由漫游模式的俯视地图测量，是 Habitat `TopDownMap` 的扩展。

---

## 8. `src/roamers/` — 自由漫游

`joy_habitat_roamer.py` — 手柄控制自由漫游，通过 ROS Service `/roam` 触发。

---

## 9. `src/scripts/` — 评估/可视化脚本

| 文件 | 功能 |
|------|------|
| `eval_and_vis_habitat.py` | 无 ROS 直接评估 + 可视化 |
| `eval_habitat_ros.py` | ROS 模式评估 |
| `compute_metrics.py` | 指标计算 |
| `compare_metrics.py` | 两组指标的 pairwise 对比 |
| `count_episodes_and_scenes.py` | 统计数据集 episode/scene 数量 |
| `visualize_episodes.py` | 可视化指定 episode |
| `visualize_metrics_from_configs.py` | 从配置可视化指标 |
| `visualize_variability_from_seeds.py` | 不同随机种子的方差分析 |
| `roam_with_joy.py` | 手柄漫游 |

---

## 10. `src/test/` — 集成测试

| 目录 | 测试内容 |
|------|---------|
| `test_habitat_ros/` | ROS 模式的 Env/Agent/Evaluator 集成测试（含 mock 组件） |
| `test_habitat/` | 无 ROS 模式 continuous/discrete 测试 |
| `test_visualization/` | 可视化测试 |
| `data/` | 测试数据 |

---

## 11. `configs/` — YAML 配置

每个配置文件定义一个完整的实验设置：
- 传感器类型 (rgb/rgbd/depth)
- 物理开关 (`pointnav_rgbd_with_physics.yaml`)
- 场景集 (HM3D/MP3D/Gibson)
- 不同 setting (2/3/4/5 — 不同的数据划分/难度)

**解耦意义:** 切换实验设置不需要改代码，换 YAML 即可。

---

## 解耦层次总结

```
┌─────────────────────────────────────────────────┐
│  evaluators/     评估编排                       │  ← 换评估模式不改下层
│  (HabitatEvaluator vs HabitatROSEvaluator)      │
├─────────────────────────────────────────────────┤
│  nodes/          ROS 中间件                     │  ← Env 和 Agent 彼此透明
│  (env_node ↔ agent_node 通过 Topic 通信)        │
├─────────────────────────────────────────────────┤
│  envs/           RL 环境抽象                    │  ← 换物理模式不改上层
│  (HabitatRLEnv dispatch: Env vs PhysicsEnv)     │
├─────────────────────────────────────────────────┤
│  tasks/          任务定义                       │  ← 离散→连续转换在此
│  (PhysicsNavigationTask._set_agent_velocities)  │
├─────────────────────────────────────────────────┤
│  sims/           模拟器底层                     │  ← 换模拟器通过 Registry
│  (HabitatSim vs HabitatPhysicsSim)              │
├─────────────────────────────────────────────────┤
│  configs/        YAML 配置                      │  ← 改参数不改代码
└─────────────────────────────────────────────────┘
```

**与 `decoup/sim_bridge/` 的关系:** `decoup/sim_bridge/` 是我们基于这个架构新建的**更轻量版本** — 去掉了 ROS 1 (`rospy`) 依赖，用 `UnifiedAction` 统一了离算/连续动作，用 `SimulatorBase` 抽象了模拟器接口。`ros_x_habitat-master` 是参考架构，`decoup/sim_bridge/` 是其现代化实现。
