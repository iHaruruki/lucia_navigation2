#!/usr/bin/env python3
import rclpy
from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import tf_transformations


def make_pose(x: float, y: float, yaw: float, frame_id: str = 'map') -> PoseStamped:
    """(x, y, yaw[rad]) から PoseStamped を生成する補助関数"""
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


def main():
    # --- Init
    rclpy.init()
    navigator = BasicNavigator()

    # --- Set initial pose（自己位置の初期値）
    initial_pose = make_pose(0.0, 0.0, 0.0)
    navigator.setInitialPose(initial_pose)

    # --- Nav2 の起動待ち
    navigator.waitUntilNav2Active()

    # === 1. 複数ウェイポイントの設定 ===
    # 例として3点のウェイポイントを用意
    waypoints = [
        # make_pose(x, y, yaw)
        make_pose(1.0, 0.0, 0.0),   # Waypoint 1
        make_pose(1.0, 1.0, 1.57),  # Waypoint 2
        make_pose(0.0, 1.0, 3.14),  # Waypoint 3
    ]

    # waypoint ナビゲーション開始
    navigator.followWaypoints(waypoints)

    # Waypoint ナビゲーションの完了待ち
    i = 0
    while not navigator.isTaskComplete():
        # 任意でフィードバックを見る
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:  # 5回に1回だけ表示など
            print(f'Currently at waypoint index: {feedback.current_waypoint}')
        i += 1

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('All waypoints reached successfully.')
    elif result == TaskResult.CANCELED:
        print('Waypoint task was canceled.')
        rclpy.shutdown()
        return
    elif result == TaskResult.FAILED:
        print('Waypoint task failed!')
        rclpy.shutdown()
        return

    # === 2. 最終ゴール（goal point）の設定 ===
    # 最後に別のゴールを設定する例
    goal_pose = make_pose(2.0, 2.0, 0.0)
    navigator.goToPose(goal_pose)

    # ゴール到達の完了待ち
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            print(f'Distance remaining: {feedback.distance_remaining:.2f} m')

    goal_result = navigator.getResult()
    if goal_result == TaskResult.SUCCEEDED:
        print('Goal reached!')
    elif goal_result == TaskResult.CANCELED:
        print('Goal navigation was canceled.')
    elif goal_result == TaskResult.FAILED:
        print('Goal navigation failed!')

    rclpy.shutdown()


if __name__ == "__main__":
    main()