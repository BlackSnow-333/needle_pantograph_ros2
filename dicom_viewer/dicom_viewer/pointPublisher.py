#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class PointPublisher(Node):
    # Publishes a point 3D position as a Float64MultiArray

    def __init__(self):
        super().__init__('point_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'image_points', 10)
        print('Point publisher initialized !')
        # timer_period = 1  # seconds
        # self.timer = self.create_timer(timer_period, self.point_publisher_callback)
    
    def publish_points(self, insertion_point, target_point):
        msg = Float64MultiArray()
        if len(insertion_point) == 0 or len(target_point) == 0 :
            msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
        else : 
            # Insertion point
            msg.data.append(insertion_point[0])
            msg.data.append(insertion_point[1])
            msg.data.append(insertion_point[2])

            # Target point
            msg.data.append(target_point[0])
            msg.data.append(target_point[1])
            msg.data.append(target_point[2])
        
        # Publish the message
        self.publisher_.publish(msg) 
        self.get_logger().info(f'Publishing insertion_point : {msg.data[0]:f}, {msg.data[1]:f}, {msg.data[2]:f}')
        self.get_logger().info(f'Publishing target_point : {msg.data[3]:f}, {msg.data[4]:f}, {msg.data[5]:f}')


def main(args=None):
    rclpy.init(args=args)
    node = PointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
