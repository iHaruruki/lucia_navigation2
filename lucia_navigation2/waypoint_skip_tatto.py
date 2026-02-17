#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Bool, String
from nav2_msgs.action import NavigateToPose
import time

class WaypointSkipNode(Node):
    def __init__(self):
        super().__init__('waypoint_skip_node')
        
        # パラメータ
        self.declare_parameter('skip_cooldown', 3.0)  # 連続スキップ防止（秒）
        self.skip_cooldown = self.get_parameter('skip_cooldown').value
        
        # サブスクライバー: /sensor_threshold_exceeded
        self.sub_threshold = self.create_subscription(
            Bool,
            '/sensor_threshold_exceeded',
            self.threshold_callback,
            10
        )
        
        # パブリッシャー: VOICEVOX TTS（オプション）
        self.pub_tts = self.create_publisher(String, '/voicevox_tts_text', 10)
        
        # Nav2 NavigateToPose アクションクライアント（現在のwaypointのみ）
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )
        
        # 状態管理
        self.last_skip_time = 0.0
        self.is_skipping = False
        
        self.get_logger().info("waypoint_skip_node を起動しました。")
        self.get_logger().info("/sensor_threshold_exceeded を監視しています。")
        self.get_logger().info("しきい値超過時、現在のwaypointをスキップして次に進みます。")
    
    def threshold_callback(self, msg: Bool):
        """しきい値超過トピックのコールバック"""
        if not msg.data:
            return
        
        # クールダウン中かチェック
        current_time = time.time()
        if current_time - self.last_skip_time < self.skip_cooldown:
            remaining = self.skip_cooldown - (current_time - self.last_skip_time)
            self.get_logger().debug(f"スキップクールダウン中（残り{remaining:.1f}秒）")
            return
        
        if self.is_skipping:
            self.get_logger().debug("既にスキップ処理中です。")
            return
        
        self.get_logger().warn("センサーしきい値超過を検知！")
        self.skip_current_waypoint()
        self.last_skip_time = current_time
    
    def skip_current_waypoint(self):
        """現在のwaypointをスキップして次に進む"""
        self.is_skipping = True
        
        self.get_logger().info("現在のwaypointをスキップしています...")
        
        # TTS送信
        self.publish_tts("危険を検知しました。次のポイントに移動します。")
        
        # NavigateToPose アクションサーバーが利用可能か確認
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("NavigateToPose アクションサーバーが利用できません。")
            self.get_logger().warn("ナビゲーションが実行中でない可能性があります。")
            self.is_skipping = False
            return
        
        # 現在のゴール（waypoint）をキャンセル
        try:
            future = self.nav_client._cancel_goal_async()
            self.get_logger().info("現在のwaypointをキャンセルしました。次のwaypointに進みます。")
        except Exception as e:
            self.get_logger().error(f"スキップ失敗: {e}")
        finally:
            self.is_skipping = False
    
    def publish_tts(self, text):
        """VOICEVOXにテキストを送信"""
        msg = String()
        msg.data = text
        self.pub_tts.publish(msg)
        self.get_logger().info(f"TTS送信: {text}")

def main(args=None):
    rclpy.init(args=args)
    node = WaypointSkipNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()