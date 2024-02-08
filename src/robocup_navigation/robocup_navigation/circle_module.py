#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MyNode(Node):
    
    def __init__(self):
        super().__init__("my_py_node")
        self.get_logger().info("TEST PYTHON")

class DrawCircleNode(Node):

    def __init__(self):
        super().__init__("draw_circle")
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer_ = self.create_timer(0.5, self.send_velocity_command)
        self.get_logger().info("Draw circle node has been started")
    
    def send_velocity_command(self):
        msg = Twist()
        msg.linear.x = 0.5
        msg.angular.z = 1.0
        self.cmd_vel_pub_.publish(msg)


