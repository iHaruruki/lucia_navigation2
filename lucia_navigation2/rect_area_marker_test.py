#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class RectAreaMarker(Node):
    def __init__(self):
        super().__init__('rect_area_marker_node')

        # Rectangle area
        self.xmin = -2.0
        self.xmax =  2.0
        self.ymin = -2.0
        self.ymax =  2.0

        self.publisher = self.create_publisher(Marker, 'rect_area_marker', 10)

        # 1 Hz
        self.timer = self.create_timer(1.0, self.publish_marker)

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = 'patrol_area'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.05

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        p1 = Point(x=self.xmin, y=self.ymin, z=0.0)
        p2 = Point(x=self.xmax, y=self.ymin, z=0.0)
        p3 = Point(x=self.xmax, y=self.ymax, z=0.0)
        p4 = Point(x=self.xmin, y=self.ymax, z=0.0)

        marker.points = [p1, p2, p3, p4, p1]

        self.publisher.publish(marker)
        self.get_logger().info('Published rect_area_marker (test node)')


def main(args=None):
    rclpy.init(args=args)
    node = RectAreaMarker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()