"""Policy node — ROS 2 Node wrapping a navigation policy.

Follows ros_x_habitat-master's habitat_agent_node.py pattern:
  - Subscribes to sensor topics (/habitat/camera_rgb, etc.)
  - Publishes actions (/habitat/plan_action)
  - Publishes lifecycle state (/ros/state)

Runs in SYSTEM Python 3.12 (rclpy). Policy implementations (PolicyImpl)
are in policy/impls.py — pure Python, no ROS dependency.

ABI note: conda Python 3.9 cannot import rclpy. This file MUST be launched
with /usr/bin/python3 (system Python). See run_eval.py::_ros_python().
"""

from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

from sim_bridge.policy.impls import PolicyImpl, get_policy_impl
from sim_bridge.policy.unified_action import ACTION

# State constants
HABITAT_STATE_READY = 0
HABITAT_STATE_ACTION_FINISH = 2
HABITAT_STATE_EPISODE_FINISH = 3


# ── ROS 2 Policy Node ──────────────────────────────────────────────────

class PolicyNode(Node):
    """ROS 2 Node wrapping a PolicyImpl.

    Like ros_x_habitat-master habitat_agent_node.py:
    - Subscribes /habitat/state for the FSM handshake
    - Subscribes /habitat/camera_rgb, /habitat/camera_depth, /habitat/odom
    - Publishes /habitat/plan_action when the FSM is ready
    - Publishes /ros/state for lifecycle

    The policy implementation (PolicyImpl) is the pluggable part.
    """

    def __init__(self, policy: PolicyImpl, node_name="policy_node"):
        super().__init__(node_name)
        self._policy = policy
        self._last_rgb = None
        self._last_depth = None
        self._last_odom = None
        self._current_state = 0
        self._step = 0

        # Subscribers (sensors)
        self.rgb_sub = self.create_subscription(
            Image, "/habitat/camera_rgb", self._on_rgb, 10)
        self.depth_sub = self.create_subscription(
            Image, "/habitat/camera_depth", self._on_depth, 10)
        self.odom_sub = self.create_subscription(
            Odometry, "/habitat/odom", self._on_odom, 10)
        self.state_sub = self.create_subscription(
            Int32, "/habitat/state", self._on_state, 10)

        # Publishers (actions + lifecycle)
        self.action_pub = self.create_publisher(Int32, "/habitat/plan_action", 10)
        self.ros_state_pub = self.create_publisher(Int32, "/ros/state", 10)

        self.get_logger().info(f"PolicyNode ready [policy={type(policy).__name__}]")

    def _on_rgb(self, msg: Image):
        self._last_rgb = msg

    def _on_depth(self, msg: Image):
        self._last_depth = msg

    def _on_odom(self, msg: Odometry):
        self._last_odom = msg

    def _on_state(self, msg: Int32):
        """FSM handshake: when env is ready for action, publish one."""
        habitat_state = msg.data

        if habitat_state == HABITAT_STATE_READY:
            # Episode starting — reset policy
            self._policy.reset()
            self._step = 0

        elif habitat_state in (HABITAT_STATE_ACTION_FINISH,):
            # Env finished executing previous action — plan next one
            self._plan_and_publish()

        elif habitat_state == HABITAT_STATE_EPISODE_FINISH:
            self.get_logger().info(f"Episode done after {self._step} steps")

    def _plan_and_publish(self):
        """Run policy, publish action as discrete Int32."""
        action = self._policy.decide(
            rgb=self._last_rgb,
            depth=self._last_depth,
            odom=self._last_odom,
            state=self._current_state,
        )
        discrete_id = action.to_discrete()

        msg = Int32(data=discrete_id)
        self.action_pub.publish(msg)

        name = ACTION.NAMES.get(discrete_id, f"?{discrete_id}")
        self._step += 1
        self.get_logger().info(
            f"Step {self._step}: {name} "
            f"(vx={action.vx:.2f}, yaw_rate={action.yaw_rate:.3f})"
        )


# ── Main ────────────────────────────────────────────────────────────────

def main():
    import argparse
    import os

    parser = argparse.ArgumentParser(description="Policy Node (ROS 2)")
    parser.add_argument("--policy", default=os.environ.get("POLICY", "random_explorer"))
    parser.add_argument("--bridge-url", default=os.environ.get("BRIDGE_URL", "http://127.0.0.1:18081"))
    args = parser.parse_args()

    rclpy.init()
    impl = get_policy_impl(args.policy, bridge_url=args.bridge_url)
    node = PolicyNode(impl)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        impl.close()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
