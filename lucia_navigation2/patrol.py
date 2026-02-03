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


class RandomRectNavigator(Node):
    def __init__(self):
        super().__init__('random_rect_navigator')
        self.navigator = BasicNavigator()

        # Rectangle area (map frame) for random goals
        self.xmin = -2.0
        self.xmax =  2.0
        self.ymin = -2.0
        self.ymax =  2.0

        # Initial pose (map frame)
        self.initial_x = 0.0
        self.initial_y = 0.0
        self.initial_yaw = 0.0  # rad

        # How many times to try reachability per cycle
        self.max_goal_sampling_tries = 20

        # Start pose for path planning (updated after each successful goal)
        self.start_pose: PoseStamped | None = None

    # -------- 1. Random goal in rectangle --------

    def sample_random_goal(self) -> PoseStamped:
        """Step 1: randomly sample a goal pose (x, y, yaw) inside the rectangle."""
        x = random.uniform(self.xmin, self.xmax)
        y = random.uniform(self.ymin, self.ymax)
        yaw = random.uniform(-math.pi, math.pi)
        return make_pose(x, y, yaw)

    # ---- 2 & 3. Use Nav2 to plan path and check reachability ----

    def get_reachable_random_goal(self) -> PoseStamped | None:
        """
        Step 2 & 3:
        - Use Nav2 getPath(start=self.start_pose, goal=random_goal).
        - If path exists, return that goal. Otherwise try another.
        """
        if self.start_pose is None:
            self.get_logger().warn(
                'start_pose is not set. Cannot check reachability.'
            )
            return None

        for i in range(self.max_goal_sampling_tries):
            goal_pose = self.sample_random_goal()

            self.get_logger().info(
                f"[Sampling {i+1}/{self.max_goal_sampling_tries}] "
                f"Random goal candidate: x={goal_pose.pose.position.x:.2f}, "
                f"y={goal_pose.pose.position.y:.2f}"
            )

            # Plan path from start_pose to candidate goal
            path = self.navigator.getPath(start=self.start_pose, goal=goal_pose)

            if path is not None and len(path.poses) > 0:
                self.get_logger().info("Path found. This random goal is reachable.")
                return goal_pose
            else:
                self.get_logger().info(
                    "No valid path for this goal. Trying another random goal..."
                )

        self.get_logger().warn(
            "Could not find any reachable random goal in the rectangle "
            "after several trials."
        )
        return None

    # ---------------------- Main loop ----------------------

    def run(self):
        # 1) Set initial pose and remember it as start_pose
        initial_pose = make_pose(self.initial_x, self.initial_y, self.initial_yaw)
        self.get_logger().info('Setting initial pose...')
        self.navigator.setInitialPose(initial_pose)
        self.start_pose = initial_pose

        # 2) Wait for Nav2 to become active
        self.get_logger().info('Waiting for Nav2 to become active...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 is now active.')

        goal_count = 0

        try:
            # 3) Main loop: 1 -> 2 -> 3 -> 4 (back to 1) until Ctrl+C
            while rclpy.ok():
                goal_count += 1
                self.get_logger().info(
                    f'========== Random navigation cycle {goal_count} =========='
                )

                # Steps 1–3: get reachable random goal
                goal_pose = self.get_reachable_random_goal()
                if goal_pose is None:
                    self.get_logger().warn(
                        'No reachable random goal found. Stopping navigation.'
                    )
                    break

                self.get_logger().info(
                    f"Executing random goal {goal_count} at "
                    f"({goal_pose.pose.position.x:.2f}, "
                    f"{goal_pose.pose.position.y:.2f})"
                )

                # Execute goal
                self.navigator.goToPose(goal_pose)

                # Wait for completion
                i = 0
                while not self.navigator.isTaskComplete():
                    feedback = self.navigator.getFeedback()
                    if feedback and i % 5 == 0:
                        self.get_logger().info(
                            f"[Goal {goal_count}] Distance remaining: "
                            f"{feedback.distance_remaining:.2f} [m]"
                        )
                        self.get_logger().info(
                            f"[Goal {goal_count}] Elapsed time: "
                            f"{feedback.navigation_time.sec} [s]"
                        )
                    i += 1

                result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info(
                        f"Random goal {goal_count} reached successfully."
                    )
                    # Update start_pose to this goal for the next cycle
                    self.start_pose = goal_pose
                elif result == TaskResult.CANCELED:
                    self.get_logger().info(
                        f"Random goal {goal_count} was canceled. Stopping."
                    )
                    break
                elif result == TaskResult.FAILED:
                    self.get_logger().info(
                        f"Random goal {goal_count} failed. Stopping."
                    )
                    break

                self.get_logger().info(
                    "Cycle finished. Returning to step 1 (sampling next random goal). "
                    "Press Ctrl+C to stop."
                )

        except KeyboardInterrupt:
            self.get_logger().info(
                'Stop requested by user (Ctrl+C). Canceling current task...'
            )
            try:
                self.navigator.cancelTask()
            except Exception as e:
                self.get_logger().warn(f'Failed to cancel task: {e}')

        self.get_logger().info('Shutting down RandomRectNavigator.')


def main(args=None):
    rclpy.init(args=args)
    node = RandomRectNavigator()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()