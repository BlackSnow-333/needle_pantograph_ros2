#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class NeedleTrajectory(Node):

    def __init__(self):
        super().__init__('needle_trajectory_marker')
        self.publisher_ = self.create_publisher(Marker, '/needle_trajectory', 5)

        # Initialize the marker
        self.marker = Marker()
        self.marker.header.frame_id = 'world'
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.marker.type = self.marker.LINE_STRIP
        self.marker.id = 0
        self.marker.action = self.marker.ADD

        # Marker parameters
        self.marker.scale.x = 0.005
        self.marker.color.r = 1.0
        self.marker.color.g = 0.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0
        self.marker.pose.position.x = 0.0
        self.marker.pose.position.y = 0.0
        self.marker.pose.position.z = 0.0

        # Marker line points
        self.marker.points = []
        # Starting point (fulcrum point coords)
        start_pt = Point()
        start_pt.x = 0.0425
        start_pt.y = 0.16056
        start_pt.z = 0.09
        self.marker.points.append(start_pt)

        # End point (arbitrary, only for testing)
        end_pt = Point()
        end_pt.x = 0.0425 - 0.02
        end_pt.y = 0.16056 - 0.02
        end_pt.z = 0.09 + 0.05

        self.marker.points.append(end_pt)
        self.get_logger().info("Publishing the needle_trajectory topic. Use RViz to visualize.")

    def publish_marker(self):
        self.publisher_.publish(self.marker)


def main(args=None):
    rclpy.init(args=args)
    trajectory = NeedleTrajectory()

    while rclpy.ok():
        trajectory.publish_marker()
    trajectory.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
