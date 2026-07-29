# ros_x_habitat-master: 每个目录如何实现解耦

## 与 decoup/sim_bridge 的映射

**不能直接复用** `ros_x_habitat-master`，因为它是 **ROS 1 (`rospy`)**，`decoup/sim_bridge/` 是**零 ROS**。但架构模式一致，每个目录都有对应物：

| ros_x_habitat-master (ROS 1) | decoup/sim_bridge 对应 (Zero ROS) | 为什么不同 |
|---|---|---|
| `evaluators/` — `HabitatROSEvaluator` 启动子进程 + ROS srv | `eval/episode_runner.py` — 直接函数调用，同一进程 | 不需要跨进程通信 |
| `nodes/habitat_env_node.py` — ROS1 Node pub `/rgb` sub `/action` | `habitat_sim_adapter.py` — 直接调 `habitat.Env` | 不需要序列化/反序列化 |
| `nodes/habitat_agent_node.py` — ROS1 Node sub sensor topics | `policy/*.py` — `PolicyBase.predict()` 直接函数调用 | Agent 就是 Python 对象，不需要 ROS topic |
| `envs/habitat_rlenv.py` — `gym.Env` 包装，physics dispatch | `sim_adapter_base.py` — `SimulatorBase`，更轻，无 gym 依赖 | 不需要 RL reward/done 抽象 |
| `envs/physics_env.py` — Bullet 物理 + LoCoBot 刚体 | 暂无（需要时加 `physics_sim_adapter.py`） | 当前只做 benchmark，不需要物理 |
| `tasks/habitat_physics_task.py` — **`_set_agent_velocities()`** | `policy/unified_action.py` — **`from_discrete()` 使用相同参数** | 提取了速度值，内嵌到 UnifiedAction |
| `sims/physics_simulator.py` — Habitat-Sim C++ wrapper | `habitat_sim_adapter.py` — 直接包 `habitat.Env` | 一层就够，不需要 registry |
| `constants/constants.py` — 枚举单独文件 | `policy/unified_action.py` — `ACTION` class 内嵌 | 简单项目不需要拆文件 |
| `test/` — rostest + mock nodes | `test/test_decoupled_e2e.py` — unittest + MockSimulator | 不需要 ROS |
| `scripts/` — 评估/可视化 | `run_eval.py` — 单入口 | 合并到一个 CLI |
| `configs/` — 20+ YAML | 直接复用 ApexNav 的 `config/` 下 Hydra YAML | 不需要自己的 config |

### 核心差异

```
ros_x_habitat-master (3 进程):
  Evaluator ──ROS srv──▶ EnvNode ──ROS topic──▶ AgentNode
                                            agent.act()
  Evaluator ◀──ROS srv── EnvNode ◀──ROS topic── AgentNode

decoup/sim_bridge (1 进程):
  EpisodeRunner ──▶ Policy.predict() ──▶ UnifiedAction ──▶ HabitatSimAdapter.step()
       ▲                                                       │
       └────────────────── observations ──────────────────────┘
  全是 Python 对象，直接函数调用，零序列化
```

### 唯一复用的代码

```python
# ros_x_habitat-master: src/tasks/habitat_physics_task.py L177-193
agent_vel_control.linear_velocity  = [0, 0, -0.25 / control_period]    # MOVE_FORWARD
agent_vel_control.angular_velocity = [0, deg2rad(10)/control_period, 0] # TURN_LEFT

# decoup/sim_bridge: policy/unified_action.py L112-116
_DISCRETE_TO_UNIFIED = {
    1: UnifiedAction(vx=0.25),                          # 同样的 0.25 m/s
    2: UnifiedAction(yaw_rate=math.radians(10.0)),      # 同样的 10°/s
}
```

---

## 完整目录说明

### 1. `src/evaluators/` — 评估编排层

**解耦:** 评估器与 ROS 的分离。

```
Evaluator (抽象) → HabitatSimEvaluator (共享逻辑)
                        ├── HabitatEvaluator (无 ROS，直接调)
                        └── HabitatROSEvaluator (有 ROS，跨进程)
```

两个子类共享完全相同的基类，上层调用 `evaluate()` 接口一致。

### 2. `src/nodes/` — ROS 中间件层

**解耦:** Env 和 Agent 是两个独立 ROS Node，通过 Topic 通信，彼此不知道对方实现。

```
HabitatEnvNode ──/rgb,/depth,/ptgoal──▶ HabitatAgentNode ← 订阅传感器
                                        agent.act() → action
HabitatEnvNode ◀──/action────────────── HabitatAgentNode ← 发布动作
                env.step(action)
```

Gazebo 桥接节点 (`gazebo_to_habitat_agent.py`, `habitat_agent_to_gazebo.py`) 证明了这个解耦 — 把 Habitat 换成 Gazebo，Agent 一行不用改。

### 3. `src/envs/` — RL 环境抽象层

**解耦:** 物理模式 vs teleport 模式。

```python
class HabitatRLEnv(gym.Env):
    def __init__(self, config, enable_physics=False):
        self._env = PhysicsEnv(config) if enable_physics else Env(config)

    def step(self, action):
        if self.enable_physics:
            return self._env.step_physics(action)   # Bullet 物理
        return self._env.step(action)               # 原生 teleport
```

一个 flag 切换。

### 4. `src/sims/` — 模拟器底层

**解耦:** 通过 Habitat Registry 注册 `"Sim-Phys"`，YAML 配置切换。

### 5. `src/tasks/` — 任务定义层

**解耦:** 离算动作 → 连续速度的核心转换。`_set_agent_velocities()` 是 `UnifiedAction.from_discrete()` 速度参数的权威来源。

### 6. `configs/` — YAML 配置

20+ 配置文件覆盖传感器类型、物理开关、场景集、数据划分。换设置只换 YAML。
