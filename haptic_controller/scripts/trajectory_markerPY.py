#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np

from std_msgs.msg import Float64MultiArray

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    Marker
)


class InteractiveMarkerNode(Node):

    def __init__(self):
        super().__init__('trajectory_marker_node')

        # self.publisher_ = self.create_publisher(
        #     Float64MultiArray,
        #     '/trajectory_marker',
        #     5
        # )

        # create an interactive marker server on the topic namespace trajectory_marker
        self.marker_server = InteractiveMarkerServer(self, 'trajectory_marker')
        # self.marker_position = np.array([0.0425, 0.16056, 0.09])

        # create an interactive marker for our server
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "world"
        int_marker.name = "trajectory_marker"
        int_marker.description = "Current operator's desired trajectory"


        # Fixed position of the arrow tip at the insertion point 
        int_marker.pose.position.x = 0.0425
        int_marker.pose.position.y = 0.16056
        int_marker.pose.position.z = 0.09
        int_marker.scale = 0.1

        # Create ARROW marker
        arrow_marker = Marker()
        arrow_marker.type = Marker.ARROW
        arrow_marker.scale.x = 0.2  # Length
        arrow_marker.scale.y = 0.005  # Arrow width
        arrow_marker.scale.z = 0.005  # Arrow height
        arrow_marker.color.r = 1.0
        arrow_marker.color.g = 0.0
        arrow_marker.color.b = 0.0
        arrow_marker.color.a = 1.0
        # arrow_marker.points.append(self.marker_position)

        # Create marker orientation controls
        control_rot_y = InteractiveMarkerControl()
        control_rot_y.name = "rotate_y"
        control_rot_y.always_visible = True
        control_rot_y.orientation.w = 1.0
        control_rot_y.orientation.x = 0.0
        control_rot_y.orientation.y = 1.0
        control_rot_y.orientation.z = 0.0
        control_rot_y.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control_rot_y.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control_rot_y)

        control_rot_z = InteractiveMarkerControl()
        control_rot_z.name = "rotate_z"
        control_rot_z.orientation.w = 1.0
        control_rot_z.orientation.x = 0.0
        control_rot_z.orientation.y = 0.0
        control_rot_z.orientation.z = 1.0
        control_rot_z.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control_rot_z.orientation_mode = InteractiveMarkerControl.INHERIT
        int_marker.controls.append(control_rot_z)

        # Create arrow control for visualization
        arrow_control = InteractiveMarkerControl()
        arrow_control.always_visible = True
        arrow_control.markers.append(arrow_marker)
        int_marker.controls.append(arrow_control)

        self.marker_server.insert(int_marker)
        self.marker_server.applyChanges()

    def process_feedback(self, feedback):
        p = feedback.pose.position
        q = feedback.pose.orientation
        self.get_logger().info('Feedback from marker: Position (%.2f, %.2f, %.2f), Orientation (%.2f, %.2f, %.2f, %.2f)' % (p.x, p.y, p.z, q.x, q.y, q.z, q.w))

        # self.marker_server.setCallback(int_marker.name, self.process_marker_feedback)

    #     # 'commit' changes and send to all clients
    #     self.marker_server.applyChanges()

    #     # Create timer to update the published marker
    #     self.timer = self.create_timer(0.002, self.update)

    # def update(self):
    #     msg = Float64MultiArray()
    #     msg.data = [self.marker_position[0], self.marker_position[1], self.marker_position[2]]
    #     self.publisher_.publish(msg)

    # def process_marker_feedback(self, feedback):
    #     self.marker_position[0] = feedback.pose.position.x
    #     self.marker_position[1] = feedback.pose.position.y
    #     self.marker_position[2] = feedback.pose.position.z
    
def main(args=None):
    rclpy.init(args=args)
    node = InteractiveMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
