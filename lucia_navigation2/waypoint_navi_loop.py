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


class LoopRouteNavigator(Node):
    def __init__(self):
        super().__init__('loop_route_navigator')
        self.navigator = BasicNavigator()

        # Define route (waypoints + final goal)
        self.waypoints = [
            # make_pose(x, y, yaw)
            make_pose(-3.41, -4.13, 0.0),    # Waypoint 1
            make_pose(-4.5, -8.86, 0.0),    # Waypoint 2
            make_pose(-0.17, -9.34, 0.0),    # Waypoint 3
            make_pose(4.06, 8.50, 3.14),    # Waypoint 4
            make_pose(2.95, -4.12, 3.14),    # Waypoint 5
            make_pose(-0.65, 0.62, 3.14),    # Waypoint 6
        ]
        self.final_goal = make_pose(2.0, 2.0, 0.0)   # Final goal of the loop

    def run(self):
        # --- Set initial pose
        # initial_pose = make_pose(0.0, 0.0, 0.0)
        # self.get_logger().info('Setting initial pose...')
        # self.navigator.setInitialPose(initial_pose)

        # --- Wait for Nav2 to become active
        self.get_logger().info('Waiting for Nav2 to become active...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 is now active.')

        loop_count = 0

        try:
            # === Main loop: keep moving on the same route until stopped ===
            while rclpy.ok():
                loop_count += 1
                self.get_logger().info(f'========== Start loop {loop_count} ==========')

                # --- 1. Follow waypoints
                self.get_logger().info('Starting waypoint navigation...')
                self.navigator.followWaypoints(self.waypoints)

                i = 0
                while not self.navigator.isTaskComplete():
                    feedback = self.navigator.getFeedback()
                    if feedback and i % 5 == 0:  # Log every 5th feedback
                        self.get_logger().info(
                            f"[Waypoints] Current waypoint index: {feedback.current_waypoint}"
                        )
                    i += 1

                result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info('All waypoints reached successfully.')
                elif result == TaskResult.CANCELED:
                    self.get_logger().info('Waypoint task was canceled. Stopping loop.')
                    break
                elif result == TaskResult.FAILED:
                    self.get_logger().info('Waypoint task failed. Stopping loop.')
                    break

                # --- 2. Go to final goal of the route
                self.get_logger().info(
                    f"Starting final goal navigation to "
                    f"({self.final_goal.pose.position.x:.2f}, "
                    f"{self.final_goal.pose.position.y:.2f})..."
                )
                self.navigator.goToPose(self.final_goal)

                j = 0
                while not self.navigator.isTaskComplete():
                    feedback = self.navigator.getFeedback()
                    if feedback and j % 5 == 0:
                        self.get_logger().info(
                            f"[Goal] Distance remaining: {feedback.distance_remaining:.2f} [m]"
                        )
                        self.get_logger().info(
                            f"[Goal] Elapsed time: {feedback.navigation_time.sec} [s]"
                        )
                    j += 1

                goal_result = self.navigator.getResult()
                if goal_result == TaskResult.SUCCEEDED:
                    self.get_logger().info('Final goal reached. Loop completed.')
                elif goal_result == TaskResult.CANCELED:
                    self.get_logger().info('Final goal was canceled. Stopping loop.')
                    break
                elif goal_result == TaskResult.FAILED:
                    self.get_logger().info('Final goal failed. Stopping loop.')
                    break

                self.get_logger().info(
                    'Loop finished. Starting next loop unless stop is requested (Ctrl+C)...'
                )

        except KeyboardInterrupt:
            # Here we also stop the robot by canceling the current Nav2 task
            self.get_logger().info('Stop requested by user (Ctrl+C). Canceling current task...')
            try:
                self.navigator.cancelTask()
            except Exception as e:
                self.get_logger().warn(f'Failed to cancel task: {e}')

        self.get_logger().info('Shutting down LoopRouteNavigator.')


def main(args=None):
    rclpy.init(args=args)
    node = LoopRouteNavigator()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()