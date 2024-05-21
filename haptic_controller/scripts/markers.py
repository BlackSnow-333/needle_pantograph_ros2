#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray


class NeedleTrajectory(Node):

    def __init__(self):
        super().__init__('needle_trajectory_marker')

        # Create subscription to topic /target_point
        self.subscription = self.create_subscription(Float64MultiArray, '/target_point',
                                                     self.listener_callback, 5)

        self.subscription  # Prevent unused variable warning

        self.publisher = self.create_publisher(Marker, '/visualization_marker', 5)

    def listener_callback(self, msg):
        # Ensure the message contains 3 elements
        if len(msg.data) != 3:
            self.get_logger().error('Received Float64MultiArray does not contain exactly 3 elements.')
            return

        # Extract the point coordinates
        x, y, z = msg.data

        # Initialize the marker
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = marker.LINE_STRIP
        marker.id = 0
        marker.action = marker.ADD

        # Marker parameters
        marker.scale.x = 0.005
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        # Marker line points
        marker.points = []
        # Starting point (fulcrum point coords)
        start_pt = Point()
        start_pt.x = 0.0
        start_pt.y = 0.16056
        start_pt.z = 0.09
        marker.points.append(start_pt)

        # Create the target point
        target_point = Point()
        target_point.x = x
        target_point.y = y
        target_point.z = z
        marker.points.append(target_point)

        self.publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    trajectory_node = NeedleTrajectory()

    try:
        rclpy.spin(trajectory_node)
    except KeyboardInterrupt:
        pass
    trajectory_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
