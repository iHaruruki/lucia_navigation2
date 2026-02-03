#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import tf_transformations
import random
import math


def make_pose(x: float, y: float, yaw: float, frame_id: str = 'map') -> PoseStamped:
    """Helper function to create PoseStamped from (x, y, yaw[rad])."""
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)

    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = rclpy.clock.Clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y
    pose.pose.orientation.z = q_z
    pose.pose.orientation.w = q_w
    return pose


class RandomPatrolNavigator(Node):
    def __init__(self):
        super().__init__('random_patrol_navigator')
        self.navigator = BasicNavigator()

        # Patrol area (map frame). Adjust these values to your environment.
        self.xmin = -2.0
        self.xmax =  2.0
        self.ymin = -2.0
        self.ymax =  2.0

        # Initial pose of the robot (map frame)
        self.initial_x = 0.0
        self.initial_y = 0.0
        self.initial_yaw = 0.0  # rad

        # How many times to try generating a reachable random goal per step
        self.max_goal_sampling_tries = 20

    # ---------- Random goal generation & reachability check ----------

    def _sample_random_goal_pose(self) -> PoseStamped:
        """Generate a random goal pose within the specified rectangular area (no reachability check)."""
        x = random.uniform(self.xmin, self.xmax)
        y = random.uniform(self.ymin, self.ymax)
        yaw = random.uniform(-math.pi, math.pi)
        return make_pose(x, y, yaw)

    def get_reachable_random_goal(self) -> PoseStamped | None:
        """
        Generate a random goal and check if it is reachable using Nav2 getPath.
        Returns a PoseStamped if reachable, otherwise None after several failed trials.
        """
        current_pose = self.navigator.getCurrentPose()
        if current_pose is None:
            self.get_logger().warn('Current pose is not available. Cannot check reachability.')
            return None

        for i in range(self.max_goal_sampling_tries):
            goal_pose = self._sample_random_goal_pose()

            self.get_logger().info(
                f"[Sampling {i+1}/{self.max_goal_sampling_tries}] "
                f"Trying random goal: x={goal_pose.pose.position.x:.2f}, "
                f"y={goal_pose.pose.position.y:.2f}"
            )

            # Ask Nav2 for a path from current pose to this random goal
            path = self.navigator.getPath(start=current_pose, goal=goal_pose)

            if path is not None and len(path.poses) > 0:
                self.get_logger().info(
                    "Found a reachable random goal (path exists). Using this goal."
                )
                return goal_pose
            else:
                self.get_logger().info(
                    "No valid path to this goal. Sampling another random goal..."
                )

        self.get_logger().warn(
            "Failed to find a reachable random goal in the given area "
            "after several tries."
        )
        return None

    # ------------------------------ Main logic ------------------------------

    def run(self):
        # --- Set initial pose
        initial_pose = make_pose(self.initial_x, self.initial_y, self.initial_yaw)
        self.get_logger().info('Setting initial pose...')
        self.navigator.setInitialPose(initial_pose)

        # --- Wait for Nav2 to become active
        self.get_logger().info('Waiting for Nav2 to become active...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 is now active.')

        goal_count = 0

        try:
            # Main loop: keep patrolling with random reachable goals until Ctrl+C
            while rclpy.ok():
                goal_count += 1
                self.get_logger().info(
                    f'========== Start random goal {goal_count} =========='
                )

                # Get a reachable random goal (checked by getPath)
                goal_pose = self.get_reachable_random_goal()
                if goal_pose is None:
                    # Could not find any reachable goal in the area
                    self.get_logger().warn(
                        "Could not find a reachable random goal. Stopping patrol."
                    )
                    break

                self.get_logger().info(
                    f"Sending goal {goal_count} to "
                    f"({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f})..."
                )
                self.navigator.goToPose(goal_pose)

                # Wait until the robot reaches the goal (or the task ends)
                i = 0
                while not self.navigator.isTaskComplete():
                    feedback = self.navigator.getFeedback()
                    if feedback and i % 5 == 0:
                        self.get_logger().info(
                            f"[Random goal {goal_count}] Distance remaining: "
                            f"{feedback.distance_remaining:.2f} [m]"
                        )
                        self.get_logger().info(
                            f"[Random goal {goal_count}] Elapsed time: "
                            f"{feedback.navigation_time.sec} [s]"
                        )
                    i += 1

                result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info(
                        f"Random goal {goal_count} reached successfully."
                    )
                elif result == TaskResult.CANCELED:
                    self.get_logger().info(
                        f"Random goal {goal_count} was canceled. Stopping patrol."
                    )
                    break
                elif result == TaskResult.FAILED:
                    self.get_logger().info(
                        f"Random goal {goal_count} failed. Stopping patrol."
                    )
                    break

                self.get_logger().info(
                    "Random goal finished. Sending next random goal "
                    "(press Ctrl+C to stop)..."
                )

        except KeyboardInterrupt:
            # When Ctrl+C is pressed, cancel the current Nav2 task to stop the robot
            self.get_logger().info(
                'Stop requested by user (Ctrl+C). Canceling current task...'
            )
            try:
                self.navigator.cancelTask()
            except Exception as e:
                self.get_logger().warn(f'Failed to cancel task: {e}')

        self.get_logger().info('Shutting down RandomPatrolNavigator.')


def main(args=None):
    rclpy.init(args=args)
    node = RandomPatrolNavigator()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()