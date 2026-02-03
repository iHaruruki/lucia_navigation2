#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
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

        # State machine flags
        self.current_goal: PoseStamped | None = None
        self.goal_count = 0
        self.navigating = False  # True while a goal is in progress

        # --- Marker publisher & timer (for RViz2 rectangle) ---
        self.marker_pub = self.create_publisher(Marker, 'rect_area_marker', 10)
        self.marker_timer = self.create_timer(1.0, self.publish_rect_marker)

        # --- Main cycle timer (navigation state machine) ---
        self.cycle_timer = self.create_timer(0.1, self.cycle_step)

        # 初期姿勢設定と Nav2 アクティブ待ちを一度だけ実行
        self.setup_done = False

    # ---------------- Marker ----------------

    def publish_rect_marker(self):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = 'patrol_area'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.05  # line width

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        p1 = Point(x=self.xmin, y=self.ymin, z=0.0)
        p2 = Point(x=self.xmax, y=self.ymin, z=0.0)
        p3 = Point(x=self.xmax, y=self.ymax, z=0.0)
        p4 = Point(x=self.xmin, y=self.ymax, z=0.0)

        marker.points = [p1, p2, p3, p4, p1]

        self.marker_pub.publish(marker)

    # ------------- Random goal sampling -------------

    def sample_random_goal(self) -> PoseStamped:
        x = random.uniform(self.xmin, self.xmax)
        y = random.uniform(self.ymin, self.ymax)
        yaw = random.uniform(-math.pi, math.pi)
        return make_pose(x, y, yaw)

    def get_reachable_random_goal(self) -> PoseStamped | None:
        if self.start_pose is None:
            self.get_logger().warn('start_pose is not set. Cannot check reachability.')
            return None

        for i in range(self.max_goal_sampling_tries):
            goal_pose = self.sample_random_goal()
            self.get_logger().info(
                f"[Sampling {i+1}/{self.max_goal_sampling_tries}] "
                f"Random goal candidate: x={goal_pose.pose.position.x:.2f}, "
                f"y={goal_pose.pose.position.y:.2f}"
            )

            path = self.navigator.getPath(start=self.start_pose, goal=goal_pose)

            if path is not None and len(path.poses) > 0:
                self.get_logger().info("Path found. This random goal is reachable.")
                return goal_pose

            self.get_logger().info(
                "No valid path for this goal. Trying another random goal..."
            )

        self.get_logger().warn(
            "Could not find any reachable random goal in the rectangle "
            "after several trials."
        )
        return None

    # ---------------- Navigation state machine ----------------

    def cycle_step(self):
        """
        この関数が 0.1 秒ごとに呼ばれる。
        - 最初の1回だけ初期化（初期姿勢設定と Nav2 アクティブ待ち）
        - その後は
          - ゴール未実行なら新しいランダムゴールをセットして goToPose
          - 実行中なら isTaskComplete を見て結果処理
        """
        # まだ Nav2 準備が終わっていなければ、ここで一度だけ実行
        if not self.setup_done:
            self.setup_nav2()
            return

        # すでにゴールを実行中なら、進捗をチェック
        if self.navigating:
            if not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                if feedback:
                    self.get_logger().info(
                        f"[Goal {self.goal_count}] Distance remaining: "
                        f"{feedback.distance_remaining:.2f} [m], "
                        f"Elapsed time: {feedback.navigation_time.sec} [s]"
                    )
                return  # まだ動いているので、次の cycle_step へ
            # タスクが終わったので結果処理
            self.handle_result()
            return

        # ここに来るのは「今ゴールを実行していない」とき
        # → 新しいランダムゴールを決めて実行を開始
        goal_pose = self.get_reachable_random_goal()
        if goal_pose is None:
            self.get_logger().warn(
                'No reachable random goal found. Waiting and trying again...'
            )
            return  # 次回の cycle_step で再トライ

        self.goal_count += 1
        self.current_goal = goal_pose
        self.get_logger().info(
            f"Executing random goal {self.goal_count} at "
            f"({goal_pose.pose.position.x:.2f}, "
            f"{goal_pose.pose.position.y:.2f})"
        )
        self.navigator.goToPose(goal_pose)
        self.navigating = True

    def setup_nav2(self):
        """初期姿勢設定と Nav2 アクティブ待ちを一度だけ行う。"""
        self.get_logger().info('Setting initial pose...')
        initial_pose = make_pose(self.initial_x, self.initial_y, self.initial_yaw)
        self.navigator.setInitialPose(initial_pose)
        self.start_pose = initial_pose

        self.get_logger().info('Waiting for Nav2 to become active...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 is now active.')

        self.setup_done = True

    def handle_result(self):
        """現在のゴールの結果を処理し、状態を更新する。"""
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(
                f"Random goal {self.goal_count} reached successfully."
            )
            # 次サイクルの start_pose を現在のゴールに更新
            if self.current_goal is not None:
                self.start_pose = self.current_goal
        elif result == TaskResult.CANCELED:
            self.get_logger().info(
                f"Random goal {self.goal_count} was canceled."
            )
        elif result == TaskResult.FAILED:
            self.get_logger().info(
                f"Random goal {self.goal_count} failed. Trying a new random goal..."
            )
        # ゴール終了処理
        self.navigating = False
        self.current_goal = None


def main(args=None):
    rclpy.init(args=args)
    node = RandomRectNavigator()
    try:
        rclpy.spin(node)  # これがタイマー・Navigator の処理をすべて回す
    except KeyboardInterrupt:
        node.get_logger().info(
            'Stop requested by user (Ctrl+C). Canceling current task...'
        )
        try:
            node.navigator.cancelTask()
        except Exception as e:
            node.get_logger().warn(f'Failed to cancel task: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()