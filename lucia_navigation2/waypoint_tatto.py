#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
import tf_transformations
import json


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
            make_pose(0.00, 0.00, 0.0),    # Waypoint 1
            #make_pose(3.94, -1.38, 0.0),    # Waypoint 2
            #make_pose(4.49, -1.33, 0.0),    # Waypoint 3
            #make_pose(4.49, 0.13, 3.14),    # Waypoint 4
        ]
        self.final_goal = make_pose(2.0, 0.0, 0.0)   # Final goal of the loop

        # ========== Sleep Detect Mode Parameters ==========
        # 睡眠検出用waypointの設定（パラメータで変更可能）
        self.declare_parameter('sleep_waypoint_x', 4.0)
        self.declare_parameter('sleep_waypoint_y', 3.0)
        self.declare_parameter('sleep_waypoint_yaw', 1.57)
        
        sleep_x = self.get_parameter('sleep_waypoint_x').value
        sleep_y = self.get_parameter('sleep_waypoint_y').value
        sleep_yaw = self.get_parameter('sleep_waypoint_yaw').value
        
        self.sleep_waypoint = make_pose(sleep_x, sleep_y, sleep_yaw)
        
        # 状態管理
        self.sleep_mode_requested = False
        self.sleep_started_detected = False
        self.waiting_for_touch = False
        self.in_sleep_mode = False
        
        # Subscribers for sleep detect mode
        self.sleep_mode_sub = self.create_subscription(
            Bool,
            '/sleep_detect_mode',
            self.sleep_mode_callback,
            10
        )
        
        # 睡眠イベントのサブスクライバ
        self.sleep_event_sub = self.create_subscription(
            String,
            '/drowsy_eye/sleep_event',
            self.sleep_event_callback,
            10
        )
        
        # 接触センサのサブスクライバ
        self.sensor_sub = self.create_subscription(
            Bool,
            '/sensor_threshold_exceeded',
            self.sensor_callback,
            10
        )
        
        self.get_logger().info('Loop Route Navigator with Sleep Detect Mode initialized')
        self.get_logger().info(
            f'Sleep waypoint set to: ({sleep_x:.2f}, {sleep_y:.2f}, {sleep_yaw:.2f})'
        )

    def sleep_mode_callback(self, msg: Bool):
        """
        /sleep_detect_mode トピックからtrueを受信
        """
        if msg.data and not self.in_sleep_mode:
            self.get_logger().info('🚨 Sleep detect mode activated!')
            self.sleep_mode_requested = True

    def sleep_event_callback(self, msg: String):
        """
        /drowsy_eye/sleep_event から睡眠イベントを受信
        sleep_started イベントを検出
        """
        try:
            # JSON文字列をパース
            event_data = json.loads(msg.data)
            event_type = event_data.get('event', '')
            
            if event_type == 'sleep_started' and self.in_sleep_mode and not self.sleep_started_detected:
                self.get_logger().info('😴 Sleep event detected: sleep_started')
                person = event_data.get('person', 'Unknown')
                timestamp = event_data.get('timestamp', 'N/A')
                self.get_logger().info(f'   Person: {person}, Timestamp: {timestamp}')
                self.sleep_started_detected = True
                
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'Failed to parse sleep event JSON: {e}')
        except Exception as e:
            self.get_logger().warn(f'Error in sleep_event_callback: {e}')

    def sensor_callback(self, msg: Bool):
        """
        /sensor_threshold_exceeded からtrueを受信（接触センサがタッチされた）
        sleep_started検出後のみ反応する（ノイズ対策）
        """
        if msg.data and self.waiting_for_touch and self.sleep_started_detected:
            self.get_logger().info('✅ Touch sensor detected! Returning to patrol mode...')
            self.waiting_for_touch = False
            self.in_sleep_mode = False
            self.sleep_started_detected = False

    def handle_sleep_detect_mode(self):
        """
        睡眠検出モードの処理
        1. 睡眠検出waypointへ移動
        2. sleep_started イベント待ち
        3. 接触センサ待ち
        4. 巡回復帰
        """
        self.in_sleep_mode = True
        self.sleep_mode_requested = False
        self.sleep_started_detected = False
        
        # 現在のナビゲーションタスクをキャンセル
        self.get_logger().info('Canceling current navigation task...')
        self.navigator.cancelTask()
        
        # キャンセルが完了するまで待機
        import time
        time.sleep(1.0)
        
        # 睡眠検出用waypointへ移動
        self.get_logger().info(
            f'Navigating to sleep detection waypoint: '
            f'({self.sleep_waypoint.pose.position.x:.2f}, '
            f'{self.sleep_waypoint.pose.position.y:.2f})...'
        )
        self.navigator.goToPose(self.sleep_waypoint)
        
        # 到着まで待機
        i = 0
        while not self.navigator.isTaskComplete():
            if i % 10 == 0:
                feedback = self.navigator.getFeedback()
                if feedback:
                    self.get_logger().info(
                        f'[Sleep Mode] Distance remaining: {feedback.distance_remaining:.2f} [m]'
                    )
            i += 1
            rclpy.spin_once(self, timeout_sec=0.1)
        
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('✅ Arrived at sleep detection waypoint')
            
            # ===== Step 1: sleep_started イベントを待つ =====
            self.get_logger().info('😴 Waiting for sleep_started event from /drowsy_eye/sleep_event...')
            
            while not self.sleep_started_detected and rclpy.ok() and self.in_sleep_mode:
                rclpy.spin_once(self, timeout_sec=0.1)
            
            if not self.sleep_started_detected:
                self.get_logger().warn('Sleep mode cancelled before sleep_started detected')
                self.in_sleep_mode = False
                return
            
            # ===== Step 2: 起こす動作を実行 =====
            self.perform_wake_up_action()
            
            # ===== Step 3: 接触センサのタッチを待機 =====
            self.waiting_for_touch = True
            self.get_logger().info('👆 Waiting for touch sensor input (/sensor_threshold_exceeded)...')
            
            while self.waiting_for_touch and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
            
            self.get_logger().info('🔄 Resuming patrol route...')
        else:
            self.get_logger().warn(f'Failed to reach sleep waypoint. Result: {result}')
            self.in_sleep_mode = False
            self.sleep_started_detected = False

    def perform_wake_up_action(self):
        """
        睡眠検出を行い、人を起こす処理
        実際の起こ���動作（音声、LED、動作など）をここに実装
        """
        self.get_logger().info('🔔 Performing wake-up action...')
        
        # TODO: 実際の起こす動作を実装
        # 例:
        # - 音声出力: os.system('espeak "Wake up please"')
        # - サウンド再生: os.system('aplay wake_up.wav')
        # - LEDの点滅: self.led_publisher.publish(...)
        # - ディスプレイ表示: self.display_publisher.publish(...)
        # - アームの動作: self.arm_controller.wake_up_motion()
        
        import time
        time.sleep(1.0)  # 起こす動作の代わり
        self.get_logger().info('Wake-up action completed.')

    def run(self):
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
                    # 睡眠検出モードのチェック
                    if self.sleep_mode_requested:
                        self.handle_sleep_detect_mode()
                        break  # 巡回を中断して睡眠検出モードへ
                    
                    feedback = self.navigator.getFeedback()
                    if feedback and i % 5 == 0:  # Log every 5th feedback
                        self.get_logger().info(
                            f"[Waypoints] Current waypoint index: {feedback.current_waypoint}"
                        )
                    i += 1
                    rclpy.spin_once(self, timeout_sec=0.1)

                # 睡眠検出モードで中断された場合、次のループへ
                if self.sleep_mode_requested or self.in_sleep_mode:
                    continue

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
                    # 睡眠検出モードのチェック
                    if self.sleep_mode_requested:
                        self.handle_sleep_detect_mode()
                        break  # 巡回を中断して睡眠検出モードへ
                    
                    feedback = self.navigator.getFeedback()
                    if feedback and j % 5 == 0:
                        self.get_logger().info(
                            f"[Goal] Distance remaining: {feedback.distance_remaining:.2f} [m]"
                        )
                        self.get_logger().info(
                            f"[Goal] Elapsed time: {feedback.navigation_time.sec} [s]"
                        )
                    j += 1
                    rclpy.spin_once(self, timeout_sec=0.1)

                # 睡眠検出モードで中断された場合、次のループへ
                if self.sleep_mode_requested or self.in_sleep_mode:
                    continue

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