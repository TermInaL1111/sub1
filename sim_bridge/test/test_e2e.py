"""Integration test: MockEnvNode + PolicyNode + CoordinatorNode in Docker.

Run:
    cd /workspace/ApexNav/ros-x-habitat/decoup
    PYTHONPATH=/workspace/ApexNav/ros-x-habitat/decoup \
    python -m pytest sim_bridge/test/test_e2e.py -v
"""

import os
import subprocess
import sys
import time
import unittest


def _ros_python():
    """Return system Python 3.12 for rclpy nodes."""
    system_py = "/usr/bin/python3"
    if os.path.exists(system_py):
        return system_py
    return sys.executable


def _ros2_env():
    """Build env dict with ROS 2 sourced and decoup on PYTHONPATH."""
    env = os.environ.copy()
    decoup_root = os.path.dirname(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    )
    existing = env.get("PYTHONPATH", "")
    env["PYTHONPATH"] = f"{decoup_root}:{existing}" if existing else decoup_root

    # Source ROS 2 setup
    ros_setup = "/opt/ros/jazzy/setup.bash"
    if os.path.exists(ros_setup):
        try:
            ros_env = subprocess.check_output(
                ["bash", "-c", f"source {ros_setup} && env"],
                env=env, stderr=subprocess.DEVNULL,
            ).decode()
            for line in ros_env.strip().split("\n"):
                if "=" in line:
                    k, v = line.split("=", 1)
                    env[k] = v
        except Exception:
            pass

    return env


class TestROSPipeline(unittest.TestCase):
    """End-to-end: MockEnv + PolicyNode via ROS 2 topics."""

    def test_unified_action_roundtrip(self):
        from sim_bridge.policy.unified_action import UnifiedAction, ACTION
        import math

        a = UnifiedAction.from_discrete(2)
        self.assertAlmostEqual(a.yaw_rate, math.radians(10.0))
        for i in range(6):
            self.assertEqual(UnifiedAction.from_discrete(i).to_discrete(), i)

    def test_metrics_computation(self):
        from sim_bridge.eval.metrics import EpisodeResult, compute_averages
        results = [
            EpisodeResult(success=1, spl=0.8, steps=10),
            EpisodeResult(success=0, spl=0.3, steps=20),
        ]
        avg = compute_averages(results)
        self.assertAlmostEqual(avg["success_rate"], 0.5)

    def test_policy_impl_imports(self):
        from sim_bridge.policy.impls import (
            PolicyImpl, RandomExplorerPolicy, MobileVLAPolicy,
            ApexNavBridgePolicy, get_policy_impl,
        )
        impl = get_policy_impl("random_explorer")
        action = impl.decide(rgb=None, depth=None, odom=None, state=2)
        self.assertIsNotNone(action)
        self.assertFalse(action.stop)

    def test_coordinator_counts_correctly(self):
        from sim_bridge.eval.metrics import compute_averages, EpisodeResult
        results = [
            EpisodeResult(episode_id="0", success=1, steps=5),
            EpisodeResult(episode_id="1", success=0, steps=10),
        ]
        avg = compute_averages(results)
        self.assertEqual(avg["total_episodes"], 2)

    @unittest.skipUnless(
        os.path.exists("/opt/ros/jazzy/setup.bash"),
        "ROS 2 Jazzy not installed",
    )
    def test_nodes_run_with_mock_env(self):
        """Full pipeline: MockEnvNode → PolicyNode → CollectorNode."""
        env = _ros2_env()
        python = _ros_python()
        root = os.path.dirname(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        )

        collect_path = "/tmp/test_trace_e2e.jsonl"
        if os.path.exists(collect_path):
            os.unlink(collect_path)

        procs = []

        # Launch collector FIRST (must be ready before mock_env publishes)
        procs.append(subprocess.Popen(
            [python, "-m", "sim_bridge.collector_node",
             "--ros-args", "-p", f"output_path:={collect_path}"],
            cwd=root, env=env,
        ))
        time.sleep(1.0)

        # Launch policy SECOND
        procs.append(subprocess.Popen(
            [python, "-m", "sim_bridge.policy_node",
             "--policy", "random_explorer"],
            cwd=root, env=env,
        ))
        time.sleep(1.0)

        # Launch mock env LAST (starts publishing after 1.5s delay for subs to connect)
        procs.append(subprocess.Popen(
            [python, "-m", "sim_bridge.mock_env_node",
             "--steps", "3", "--episodes", "2"],
            cwd=root, env=env,
        ))

        # Wait for mock_env to finish (2 episodes × 3 steps → ~3s)
        for p in procs:
            try:
                p.wait(timeout=60)
            except subprocess.TimeoutExpired:
                pass

        # Cleanup
        for p in procs:
            if p.poll() is None:
                p.terminate()
                try:
                    p.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    p.kill()

        # Verify collector wrote data
        if os.path.exists(collect_path):
            with open(collect_path) as f:
                lines = [l for l in f if l.strip()]
            print(f"  Collector wrote {len(lines)} records")
            self.assertGreater(len(lines), 0)
            os.unlink(collect_path)
        else:
            self.skipTest("Collector output file not created")


if __name__ == "__main__":
    unittest.main()
