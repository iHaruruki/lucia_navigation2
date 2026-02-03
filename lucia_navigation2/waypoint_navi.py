#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy  
from rclpy.node import Node  
import sys   
import time  
from math import pi  
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped  # 位置や姿勢に関するメッセージ
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult  # Nav2 Simple Commander用
import tf_transformations  # 回転の変換を行うライブラリ

class WayPointNavi(Node):
    def __init__(self):
        super().__init__('waypoint_navi')  # ノードの初期化
        self.wp_num = 0                    # ウェイポイント番号の初期化
        self.init_pose = [-2.0, -0.5, 0.0] # 初期姿勢（x, y, yaw）
        self.navigator = BasicNavigator()  # BasicNavigatorインスタンスの作成
        
    def do_navigation(self):  ### ナビゲーションを実行するメソッド ###
        way_point = [         # ウェイポイントのリスト
            [1.2, -1.5, pi/2], [1.0, 0.5, pi], [-4.0, 0.8, pi/2], [-4.0, 3.9, pi], 
            [-6.5, 4.0, -pi/2], [-6.5, -3.0, pi/2], [999.9, 0.0, 0.0] 
        ]        
        self.set_init_pose()  # 初期姿勢の設定 
        self.navigator.waitUntilNav2Active()            # Nav2がアクティブになるまで待機
        while rclpy.ok():     # ナビゲーションのメインループ 
            if way_point[self.wp_num][0] == 999.9:      # 終了条件のチェック
                self.get_logger().info('ナビゲーションを終了します．')
                sys.exit(0)                             # プログラムの正常終了
            pose_msg = self.to_pose_msg(way_point[self.wp_num])  # 現在のウェイポイントの姿勢を設定
            result = self.navigate_to_goal(pose_msg)    # ゴールへナビゲーション
            time.sleep(1)  

    def set_init_pose(self):  ### Nav2に初期姿勢を設定するメソッド ### 
        init_pose_msg = self.to_pose_msg(self.init_pose)   # メッセージ型に変換     
        self.get_logger().info('初期位置を設定します．')
        self.navigator.setInitialPose(init_pose_msg)       # Nav2に初期姿勢を設定        

    def navigate_to_goal(self, goal_pose):  ### ゴールへナビゲーション ###
        self.get_logger().info(f"WP{self.wp_num + 1}({goal_pose.pose.position.x},{goal_pose.pose.position.y})に行きます．")
        self.navigator.goToPose(goal_pose)            # ゴールを指定してナビゲーションを開始
        while not self.navigator.isTaskComplete():    # タスクの完了を待つループ
            feedback = self.navigator.getFeedback()   # フィードバックの取得
            if feedback:
                self.get_logger().info(f"残り：{feedback.distance_remaining:.2f}[m]")
                self.get_logger().info(f"経過時間：{feedback.navigation_time.sec}[s]") 
                if feedback.navigation_time.sec > 99: # ナビゲーションの経過時間が超過すると 
                    self.navigator.cancelTask()       # タスクをキャンセル
            time.sleep(0.5)                           # フィードバックを取得する間隔
        result = self.navigator.getResult()           # 結果の取得        
        if result == TaskResult.SUCCEEDED:            # 成功した場合
            self.get_logger().info(f'WP{self.wp_num + 1}に着きました．')
            self.wp_num += 1                          # 次のウェイポイントに進む
        elif result == TaskResult.CANCELED:           # キャンセルされた場合
            self.get_logger().info(f"WP{self.wp_num + 1}はキャンセルされました．")
            self.wp_num += 1                          # 一つ飛ばして次のウェイポイントに進む
        else:                                         # 失敗した場合
            self.get_logger().info(f'WP{self.wp_num + 1}は失敗しました．')
            sys.exit(1)                               # プログラムの異常終了        
    
    def to_pose_msg(self, pose):  ### メッセージ型に変換するメソッド ###  
        pose_msg = PoseStamped()               # ウェイポイントの姿勢を設定
        pose_msg.header.stamp = self.navigator.get_clock().now().to_msg()  # 現在時間
        pose_msg.header.frame_id = "map"       # フレームIDの設定
        pose_msg.pose.position.x = pose[0]     # x座標
        pose_msg.pose.position.y = pose[1]     # y座標
        q = tf_transformations.quaternion_from_euler(0, 0, pose[2])  # 角度の変換
        pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, \
        pose_msg.pose.orientation.z, pose_msg.pose.orientation.w = q
        return pose_msg


def main(args=None):
    rclpy.init(args=args)
    waypoint_navi = WayPointNavi()
    waypoint_navi.do_navigation()
    rclpy.spin(waypoint_navi)
    waypoint.destroy_node()
    rclpy.shutdown()