# Code Review: decoup/ 从零启动失败分析

## 错误日志

```
09:56:36 Simulator phase: starting habitat
09:56:38 $ docker compose run --rm shell bash -lc 'cd /workspace/LuxiNav && export
           DISPLAY=:99 PYTHONPATH=.:ros-x-habitat:$PYTHONPATH && ...'

09:56:39 /opt/conda/envs/LuxiNav/bin/python3: Error while finding module specification
          for 'sim_bridge.habitat_env_node' (ModuleNotFoundError: No module named 'sim_bridge')
09:56:39 [simulator] exited with 1
```

## 根因分析（3 个 Bug）

### Bug 1: 工作目录路径错误

```
server.py 发送的命令:          cd /workspace/LuxiNav && ...
Docker compose mount:          ./ros-x-habitat:/workspace/ApexNav/ros-x-habitat
```
`server.py` 用 `/workspace/LuxiNav`，但 compose.yaml 把 `ros-x-habitat` 挂载到 `/workspace/ApexNav/ros-x-habitat`。路径不匹配，PYTHONPATH 解析不到 `ros-x-habitat/` 目录。

**修复**: `server.py` 的 `simulator_eval_command()` 中 `cd /workspace/LuxiNav` → `cd /workspace/ApexNav`。

### Bug 2: Conda 环境名错误

```
Docker 内 conda env:           apexnav
前端日志报错的 Python:          /opt/conda/envs/LuxiNav/bin/python3
```
`LuxiNav` 是旧项目名。Docker 镜像里只有 `apexnav` conda 环境。`LuxiNav` 不存在。

**修复**: 确认 Docker 镜像的 `apexnav-entrypoint.bash` 中 `CONDA_ENV=apexnav`。如果镜像是 LuxiNav 专属构建的，需要确认 conda env 名。

### Bug 3: `sim_bridge.habitat_env_node` 不在 decoup/ 里

```
server.py 命令:    python3 -m sim_bridge.habitat_env_node
sim_bridge 在哪里:  ros-x-habitat/sim_bridge/habitat_env_node.py  ← 父级目录, 存在
                  ros-x-habitat/decoup/sim_bridge/               ← 没有 habitat_env_node
```
`habitat_env_node.py` 在**父级** `ros-x-habitat/sim_bridge/`，不在 `decoup/sim_bridge/`。PYTHONPATH 需要包含 `ros-x-habitat`（父级），不是 `ros-x-habitat/decoup`。

当前 `simulator_eval_command()` 设 `PYTHONPATH=.:ros-x-habitat:$PYTHONPATH` — 这是对的（`ros-x-habitat/sim_bridge/` 可被找到）。但 Bug 1 导致路径错误，所以实际不可用。

**修复**: 修 Bug 1 后，这个也就自动好了。

---

## 修复清单

### 1. server.py `simulator_eval_command()` (line 2596)

```python
# 当前 (错误):
"cd /workspace/LuxiNav && export DISPLAY=:99 PYTHONPATH=.:ros-x-habitat:$PYTHONPATH && "

# 修复:
"cd /workspace/ApexNav && export DISPLAY=:99 PYTHONPATH=.:ros-x-habitat:$PYTHONPATH && "
```

### 2. 确认 compose.yaml 的 sim-node 服务

```yaml
# compose.yaml 中已有一个 sim-node 定义
sim-node:
  command: ["bash", "/workspace/ApexNav/ros-x-habitat/sim_bridge/entrypoint.sh"]
```

这个 entrypoint.sh 会对非 habitat 后端用 `/usr/bin/python3`，对 habitat 后端用 conda python。但 server.py 没用它——它自己拼了 `docker compose run --rm shell`。

**建议**: server.py 改用 `docker compose run --rm sim-node`（如果 sim-node 服务定义完整），或者确保 `shell` 服务的 conda env 名正确。

### 3. decoup policy_node 的 PYTHONPATH

decoup `policy_node.py` 在 `decoup/sim_bridge/` 下。它 import `sim_bridge.policy.unified_action` — 需要 `decoup/` 在 PYTHONPATH。

当前 server.py line 2694 已经做了：
```python
existing = env.get("PYTHONPATH", "")
env["PYTHONPATH"] = f"{decoup_root}:{existing}" if existing else decoup_root
```

确认 `decoup_root` 指向 `ros-x-habitat/decoup`（而不是 `decoup/sim_bridge`）。

---

## 验证步骤

```bash
# 1. 确认 Docker 内 sim_bridge 可 import
sudo docker compose run --rm shell bash -lc "
  cd /workspace/ApexNav
  PYTHONPATH=.:ros-x-habitat python3 -c 'import sim_bridge; print(sim_bridge.__file__)'
"
# 预期输出: /workspace/ApexNav/ros-x-habitat/sim_bridge/__init__.pyc

# 2. 确认 conda env 名
sudo docker compose run --rm shell bash -lc "
  conda info --envs | grep -E 'apexnav|LuxiNav'
"
# 预期输出: apexnav  * /opt/conda/envs/apexnav

# 3. 确认 decoup 可 import
sudo docker compose run --rm shell bash -lc "
  cd /workspace/ApexNav
  PYTHONPATH=ros-x-habitat/decoup python3 -c 'from sim_bridge.policy.unified_action import UnifiedAction; print(UnifiedAction.from_discrete(1))'
"

# 4. decoup policy_node 独立启动测试
sudo docker compose run --rm shell bash -lc "
  source /opt/ros/jazzy/setup.bash
  source install/setup.bash
  cd /workspace/ApexNav/ros-x-habitat/decoup
  PYTHONPATH=. /usr/bin/python3 -m sim_bridge.policy_node --policy random_explorer &
  sleep 2
  /usr/bin/python3 -m sim_bridge.mock_env_node --steps 3 --episodes 1
"
```
