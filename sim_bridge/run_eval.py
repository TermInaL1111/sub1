#!/usr/bin/env python3
"""Launch the 3 nodes of a decoupled evaluation pipeline.

Like ros_x_habitat-master's HabitatROSEvaluator which spawns env_node + agent_node
as subprocesses, this script launches:

  1. Mock Env Node (or real Habitat via ros2_http_bridge)
  2. Policy Node (random_explorer / apexnav / mobilevla)
  3. Coordinator Node (monitors state, collects metrics)
  4. Collector Node (optional, writes JSONL trace)

All three are native ROS 2 rclpy Nodes. The env side (Habitat) is started
separately via ros2_http_bridge.py + habitat_wrapper.py in conda.

Usage:
    # Test with mock env (no Habitat needed):
    python -m sim_bridge.run_eval --mock --policy random_explorer --episodes 3

    # Real Habitat eval (requires ros2_http_bridge on port 18080):
    python -m sim_bridge.run_eval --policy random_explorer --episodes 5

    # With data collection:
    python -m sim_bridge.run_eval --mock --collect /tmp/trace.jsonl
"""

import argparse
import os
import subprocess
import sys
import time


def _ros2_env():
    """Build env dict for ROS 2 subprocess."""
    env = os.environ.copy()
    # Add decoup/ to PYTHONPATH so sim_bridge is importable
    decoup_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    existing = env.get("PYTHONPATH", "")
    env["PYTHONPATH"] = f"{decoup_root}:{existing}" if existing else decoup_root
    return env


def _ros_python():
    """Return the system Python 3.12 binary (needed for rclpy).

    Conda Python 3.9 cannot import rclpy. ROS 2 nodes MUST run in system Python.
    """
    system_py = "/usr/bin/python3"
    if os.path.exists(system_py):
        return system_py
    # Fallback: try current python if it can import rclpy
    try:
        import rclpy  # noqa: F401
        return sys.executable
    except ImportError:
        raise RuntimeError(
            "Cannot find system Python 3.12 for rclpy. "
            "Are you running inside the Docker container?"
        )


def _launch_node(script, args, env):
    """Launch a ROS 2 Python node as subprocess using system Python."""
    cmd = [
        _ros_python(), "-m", f"sim_bridge.{script}",
    ] + args
    print(f"  Launch: {' '.join(cmd)}")
    return subprocess.Popen(cmd, env=env)


def run_mock(args):
    """Launch MockEnvNode + PolicyNode + CoordinatorNode.

    All three run in the same Python process group as ROS 2 nodes.
    """
    env = _ros2_env()

    # Launch nodes as subprocesses (each one does rclpy.init() in its own process)
    procs = []

    # 1. Mock Env
    procs.append(_launch_node(
        "mock_env_node",
        [f"--steps={args.steps_per_episode}", f"--episodes={args.episodes}"],
        env,
    ))
    time.sleep(0.5)

    # 2. Policy
    procs.append(_launch_node(
        "policy_node",
        [f"--policy={args.policy}"],
        env,
    ))
    time.sleep(0.5)

    # 3. Coordinator
    procs.append(_launch_node(
        "coordinator_node",
        [f"--ros-args", "-p", f"max_episodes:={args.episodes}"],
        env,
    ))

    # 4. Collector (optional)
    if args.collect:
        time.sleep(0.3)
        procs.append(_launch_node(
            "collector_node",
            ["--ros-args", "-p", f"output_path:={args.collect}"],
            env,
        ))

    # Wait for coordinator to finish
    try:
        for p in procs:
            p.wait(timeout=120)
    except subprocess.TimeoutExpired:
        print("Timeout — killing processes")
    finally:
        for p in procs:
            if p.poll() is None:
                p.terminate()
                try:
                    p.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    p.kill()


def run_habitat(args):
    """Real Habitat eval: ros2_http_bridge + habitat_wrapper + policy + coordinator.

    The ros2_http_bridge and habitat_wrapper are expected to be running already
    (started via docker compose or manually). This script only starts policy
    and coordinator nodes.
    """
    env = _ros2_env()
    procs = []

    # 1. Policy
    procs.append(_launch_node(
        "policy_node",
        [f"--policy={args.policy}"],
        env,
    ))
    time.sleep(0.5)

    # 2. Coordinator
    procs.append(_launch_node(
        "coordinator_node",
        [f"--ros-args", "-p", f"max_episodes:={args.episodes}"],
        env,
    ))

    if args.collect:
        time.sleep(0.3)
        procs.append(_launch_node(
            "collector_node",
            ["--ros-args", "-p", f"output_path:={args.collect}"],
            env,
        ))

    try:
        for p in procs:
            p.wait(timeout=600)
    except subprocess.TimeoutExpired:
        print("Timeout — killing processes")
    finally:
        for p in procs:
            if p.poll() is None:
                p.terminate()
                try:
                    p.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    p.kill()


# ── CLI ─────────────────────────────────────────────────────────────────

def main():
    p = argparse.ArgumentParser(description="Decoupled eval — ROS 2 pipeline launcher")
    p.add_argument("--mock", action="store_true",
                   help="Use MockEnvNode (no Habitat needed)")
    p.add_argument("--policy", default=os.environ.get("POLICY", "random_explorer"),
                   help="Policy: random_explorer, apexnav, mobilevla")
    p.add_argument("--episodes", type=int,
                   default=int(os.environ.get("EPISODES", "5")),
                   help="Number of episodes")
    p.add_argument("--steps-per-episode", type=int, default=5,
                   help="Steps per episode (mock mode only)")
    p.add_argument("--collect", default="",
                   help="JSONL output path for data collection")
    args = p.parse_args()

    print(f"Policy: {args.policy} | Episodes: {args.episodes}")
    print()

    if args.mock:
        run_mock(args)
    else:
        run_habitat(args)


if __name__ == "__main__":
    main()
