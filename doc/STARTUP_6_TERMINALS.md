# decoup 全栈启动：从前端出发

## 单命令启动

```bash
cd /home/eikom/ApexNav/luxinav-interactive-web-20260723
APEXNAV_ROOT=/home/eikom/ApexNav \
APEXNAV_INIT_SCAN_MODE=demo_horizontal \
python3 server.py --host 0.0.0.0 --port 8088 \
  --decoup-policy random_explorer \
  --decoup-collect /tmp/frontend_trace.jsonl
```

浏览器打开 **http://127.0.0.1:8088/**，输入目标，点击 Start。

`server.py` 自动按顺序启动所有组件：

```
server.py 启动
├── 1. VLM:         docker compose up -d vlm
├── 2. Bridge:      ros2_http_bridge :18081 (Docker)
├── 3. C++ FSM:     docker compose run --rm algorithm     ← 照样启动, 不受影响
├── 4. decoup policy_node (host, /usr/bin/python3)       ← 附加, 也发 /habitat/plan_action
├── 5. decoup collector_node (host, /usr/bin/python3)    ← 附加, 只订阅不发布
├── 6. Simulator:   habitat_env_node (Docker, conda)
└── 7. BridgeProxy: :18080 (内部, 截获帧存 FrameStore)
```

**注意**: C++ FSM 和 decoup policy_node **都**发布 `/habitat/plan_action`。同时运行会冲突。默认用 C++ FSM，decoup 的 policy_node 可关掉（`--decoup-policy ""` 或不传此参数）。collector_node 是纯订阅，不影响任何功能，可常驻。

Ctrl+C 停止全部。

---

## `server.py` 改动摘要

4 处改动，不改现有逻辑：

### 1. 新增 CLI 参数

```python
--decoup-policy     # 策略名 (空=不启动), 默认 ""
--decoup-collect    # JSONL 输出路径 (空=不启动), 默认 ""
--decoup-root       # decoup 目录路径
```

### 2. `__init__` 初始化

```python
self._decoup_procs: list = []   # 跟踪 decoup 子进程
```

### 3. `_start_decoup_nodes()` 方法

在 `_simulator_worker` 的 `_maybe_start_bridge_proxy()` 之后调用。用 `/usr/bin/python3` 启动 `policy_node` 和 `collector_node`。

### 4. `shutdown()` 清理

先终止 decoup 子进程，再停 bridge + oneoff containers。

---

## 手动 6 终端（备选，调试用）

```bash
# 终端 1: VLM
cd /home/eikom/ApexNav && sudo docker compose up vlm

# 终端 2: ros2_http_bridge
cd /home/eikom/ApexNav
sudo docker compose run --rm shell bash -lc "
  source /opt/ros/jazzy/setup.bash && source install/setup.bash
  /usr/bin/python3 -m habitat2ros.ros2_http_bridge --port 18081
"

# 终端 3: C++ FSM (ApexNav planner)
cd /home/eikom/ApexNav
APEXNAV_INIT_SCAN_MODE=demo_horizontal sudo docker compose up algorithm

# 终端 4: decoup collector_node (可选, 只订阅)
cd /home/eikom/ApexNav/ros-x-habitat/decoup
PYTHONPATH=. /usr/bin/python3 -m sim_bridge.collector_node \
  --ros-args -p output_path:=/tmp/frontend_trace.jsonl

# 终端 5: Habitat driver
cd /home/eikom/ApexNav
sudo docker compose run --rm shell bash -lc "
  conda activate apexnav && cd /workspace/ApexNav
  export PYTHONPATH=.:ros-x-habitat/decoup:\$PYTHONPATH
  python -m sim_bridge.habitat_env_node --config-name habitat_eval_hm3dv2 \
    --max-steps 500 --bridge-url http://127.0.0.1:18080
"

# 终端 6: 前端
cd /home/eikom/ApexNav/luxinav-interactive-web-20260723
APEXNAV_ROOT=/home/eikom/ApexNav python3 server.py --host 0.0.0.0 --port 8088
```

---

## 验证

```bash
# bridge
curl -s http://127.0.0.1:18080/health  # → {"ok":true,"proxy":true}
curl -s http://127.0.0.1:18081/health  # → {"ok":true}

# ROS topics
sudo docker compose exec shell bash -lc "
  source /opt/ros/jazzy/setup.bash
  ros2 topic list
"

# collector 数据
cat /tmp/frontend_trace.jsonl | head -3

# 停止
sudo docker compose down
pkill -f policy_node; pkill -f collector_node; pkill -f server.py
```
