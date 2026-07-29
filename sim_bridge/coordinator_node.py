"""Coordinator Node — ROS 2 evaluation orchestrator.

Like ros_x_habitat-master's HabitatROSEvaluator:
  - Monitors /habitat/state for episode lifecycle
  - Collects per-episode metrics
  - Can be extended to spawn env/policy processes

Usage:
    ros2 run sim_bridge coordinator_node --ros-args -p max_episodes:=10
"""

import os
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int32

from sim_bridge.eval.metrics import EpisodeResult, compute_averages

# State constants
HABITAT_STATE_READY = 0
HABITAT_STATE_ACTION_EXEC = 1
HABITAT_STATE_ACTION_FINISH = 2
HABITAT_STATE_EPISODE_FINISH = 3


class CoordinatorNode(Node):
    """Monitors evaluation state, collects per-episode metrics.

    Subscribes:
      /habitat/state         — episode lifecycle
      /habitat/plan_action   — action log (for step counting)
      /blip2/cosine_score    — optional VLM score

    Parameters:
      max_episodes — stop after N episodes (default: 10)
    """

    def __init__(self, node_name="coordinator"):
        super().__init__(node_name)

        self.declare_parameter("max_episodes", 10)
        self._max_episodes = self.get_parameter("max_episodes").value

        # Subscribers
        self.state_sub = self.create_subscription(
            Int32, "/habitat/state", self._on_state, 10)
        self.action_sub = self.create_subscription(
            Int32, "/habitat/plan_action", self._on_action, 10)

        # State tracking
        self._results = []
        self._episode_count = 0
        self._step_count = 0
        self._current_state = HABITAT_STATE_READY
        self._done = False

        self.get_logger().info(
            f"CoordinatorNode ready [max_episodes={self._max_episodes}]")

    def _on_action(self, msg: Int32):
        if self._current_state == HABITAT_STATE_ACTION_FINISH:
            self._step_count += 1

    def _on_state(self, msg: Int32):
        new_state = msg.data
        self._current_state = new_state

        if new_state == HABITAT_STATE_READY:
            # New episode starting
            self._step_count = 0

        elif new_state == HABITAT_STATE_EPISODE_FINISH:
            # Episode done — collect metrics
            self._episode_count += 1
            result = EpisodeResult(
                episode_id=str(self._episode_count),
                success=0,  # actual metrics come from env
                steps=self._step_count,
            )
            self._results.append(result)

            avg = compute_averages(self._results)
            self.get_logger().info(
                f"Episode {self._episode_count}/{self._max_episodes} done. "
                f"steps={self._step_count} "
                f"success_rate={avg['success_rate']:.1%}"
            )

            if self._episode_count >= self._max_episodes:
                self._print_final_summary()
                self._done = True

    def _print_final_summary(self):
        avg = compute_averages(self._results)
        self.get_logger().info(
            f"=== Evaluation Complete ===\n"
            f"  Episodes:     {avg['total_episodes']}\n"
            f"  Success:      {avg['total_success']}/{avg['total_episodes']}\n"
            f"  Success rate: {avg['success_rate']:.1%}\n"
            f"  Avg SPL:      {avg['avg_spl']:.3f}\n"
            f"  Avg steps:    {avg['avg_steps']:.1f}\n"
            f"  Avg dist2goal:{avg['avg_distance_to_goal']:.3f}"
        )

    @property
    def done(self) -> bool:
        return self._done

    @property
    def results(self) -> list:
        return list(self._results)


def main():
    rclpy.init()
    node = CoordinatorNode()

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
