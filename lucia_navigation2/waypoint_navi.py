#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import tf_transformations


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


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        self.navigator = BasicNavigator()

    def run(self):
        # --- Set initial pose
        initial_pose = make_pose(0.0, 0.0, 0.0)
        self.get_logger().info('Setting initial pose...')
        self.navigator.setInitialPose(initial_pose)

        # --- Wait for Nav2 to become active
        self.get_logger().info('Waiting for Nav2 to become active...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 is now active.')

        # === 1. Set multiple waypoints ===
        waypoints = [
            # make_pose(x, y, yaw)
            make_pose(3.76, 1.07, 0.0),     # Waypoint 1
            make_pose(3.76, -0.02, 1.57),   # Waypoint 2
            make_pose(2.5, -1.4, 3.14),     # Waypoint 3
        ]

        self.get_logger().info('Starting waypoint navigation...')
        self.navigator.followWaypoints(waypoints)

        # Wait until waypoint navigation is complete
        i = 0
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:  # Log every 5th feedback
                # FollowWaypoints_Feedback has only current_waypoint (and maybe similar),
                # so we only log the index here.
                self.get_logger().info(
                    f"[Waypoints] Current waypoint index: {feedback.current_waypoint}"
                )
            i += 1

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('All waypoints reached successfully.')
        elif result == TaskResult.CANCELED:
            self.get_logger().info('Waypoint task was canceled.')
            return
        elif result == TaskResult.FAILED:
            self.get_logger().info('Waypoint task failed!')
            return

        # === 2. Set final goal point (use goToPose, which provides distance/time feedback) ===
        goal_pose = make_pose(2.0, 2.0, 0.0)
        self.get_logger().info(
            f"Starting final goal navigation to "
            f"({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f})..."
        )
        self.navigator.goToPose(goal_pose)

        # Wait until the final goal is reached
        j = 0
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback and j % 5 == 0:  # Log every 5th feedback
                # For goToPose, feedback has distance_remaining and navigation_time.
                self.get_logger().info(
                    f"[Goal] Distance remaining: {feedback.distance_remaining:.2f} [m]"
                )
                self.get_logger().info(
                    f"[Goal] Elapsed time: {feedback.navigation_time.sec} [s]"
                )
            j += 1

        goal_result = self.navigator.getResult()
        if goal_result == TaskResult.SUCCEEDED:
            self.get_logger().info('Goal reached!')
        elif goal_result == TaskResult.CANCELED:
            self.get_logger().info('Goal navigation was canceled.')
        elif goal_result == TaskResult.FAILED:
            self.get_logger().info('Goal navigation failed!')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()