"""Collector Node — ROS 2 data collector for training traces.

Subscribes to /habitat/* topics and writes per-step JSONL records.
Like ros_x_habitat-master's write_record() but as a live ROS subscriber.

Usage:
    ros2 run sim_bridge collector_node --ros-args -p output_path:=/tmp/trace.jsonl
"""

import json
import os

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Int32


class CollectorNode(Node):
    """Subscribe to observation/action topics, write JSONL training traces.

    Parameters:
      output_path — JSONL file path (default: /tmp/training_trace.jsonl)
    """

    def __init__(self, node_name="collector"):
        super().__init__(node_name)

        self.declare_parameter("output_path", "/tmp/training_trace.jsonl")
        output_path = self.get_parameter("output_path").value

        parent = os.path.dirname(output_path)
        if parent:
            os.makedirs(parent, exist_ok=True)
        self._file = open(output_path, "a", buffering=1)
        self._record_count = 0
        self._output_path = output_path

        # Episode state
        self._episode_id = "?"
        self._step = 0
        self._last_action = 0
        self._last_position = [0.0, 0.0, 0.0]
        self._success = 0

        # Subscribers
        self.state_sub = self.create_subscription(
            Int32, "/habitat/state", self._on_state, 10)
        self.action_sub = self.create_subscription(
            Int32, "/habitat/plan_action", self._on_action, 10)
        self.odom_sub = self.create_subscription(
            Odometry, "/habitat/odom", self._on_odom, 10)

        self.get_logger().info(f"CollectorNode writing to {output_path}")

    def _on_state(self, msg: Int32):
        state = msg.data
        if state == 0:  # READY
            self._step = 0
            self._episode_id = f"ep{int(self._episode_id[2:]) + 1 if self._episode_id.startswith('ep') and self._episode_id[2:].isdigit() else '1'}"
        elif state == 2:  # ACTION_FINISH — step complete, write record
            self._flush_record()

    def _on_action(self, msg: Int32):
        self._last_action = msg.data

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self._last_position = [p.x, p.y, p.z]

    def _flush_record(self):
        record = {
            "episode_id": self._episode_id,
            "step": self._step,
            "action": self._last_action,
            "position": self._last_position,
            "success": self._success,
        }
        self._file.write(json.dumps(record) + "\n")
        self._record_count += 1
        self._step += 1

    def close(self):
        if self._file and not self._file.closed:
            self._file.close()
        self.get_logger().info(
            f"CollectorNode: {self._record_count} records → {self._output_path}"
        )


def main():
    rclpy.init()
    node = CollectorNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
