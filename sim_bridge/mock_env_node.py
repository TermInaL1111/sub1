"""Mock Env Node — ROS 2 Node simulating a Habitat environment for testing.

Publishes synthetic /habitat/state transitions so policy + coordinator
can be tested without Habitat/conda/HTTP bridge.

Like ros_x_habitat-master's mock_env_node.py test fixture.
"""

import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32


class MockEnvNode(Node):
    """Simulates the state machine an env wrapper would produce.

    Cycle: READY → ACTION_EXEC → ACTION_FINISH → ... → EPISODE_FINISH → READY
    """

    def __init__(self, node_name="mock_env", steps_per_episode=5, num_episodes=3):
        super().__init__(node_name)
        self._steps_per_episode = steps_per_episode
        self._num_episodes = num_episodes
        self._episode = 0
        self._step = 0

        # Publishers
        self.state_pub = self.create_publisher(Int32, "/habitat/state", 10)
        self.odom_pub = self.create_publisher(Odometry, "/habitat/odom", 10)

        self.get_logger().info(
            f"MockEnvNode: {num_episodes} episodes × {steps_per_episode} steps"
        )

    def run(self):
        """Blocking run: publish state transitions until done."""
        # Wait for subscribers to connect
        time.sleep(1.5)
        for epi in range(self._num_episodes):
            self._episode = epi
            self._step = 0

            # READY
            self.state_pub.publish(Int32(data=0))
            self.get_logger().info(f"Episode {epi + 1}: READY")
            time.sleep(0.1)

            for s in range(self._steps_per_episode):
                self._step = s

                # ACTION_EXEC — env is executing the previous action
                self.state_pub.publish(Int32(data=1))
                time.sleep(0.1)

                # ACTION_FINISH — env done, policy should publish next action now
                self.state_pub.publish(Int32(data=2))
                time.sleep(0.1)

            # EPISODE_FINISH
            self.state_pub.publish(Int32(data=3))  # EPISODE_FINISH
            self.get_logger().info(
                f"Episode {epi + 1}: EPISODE_FINISH after {self._steps_per_episode} steps"
            )
            time.sleep(0.2)

        self.get_logger().info("MockEnvNode done")
        return True


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--steps", type=int, default=5)
    parser.add_argument("--episodes", type=int, default=3)
    args = parser.parse_args()

    rclpy.init()
    node = MockEnvNode(
        steps_per_episode=args.steps,
        num_episodes=args.episodes,
    )
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
