#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg      import Point, Twist

class FollowMarker(Node):
    def __init__(self):
        super().__init__('follow_marker')
        self.get_logger().info('Follow Marker started')
        self.sub_ = self.create_subscription(Point,"/marker_pos",self.listener,10)
        self.pub_ = self.create_publisher(Twist,"/cmd_vel",10)
        
        self.declare_parameter("angular_speed_multiplier", 0.9)
        self.declare_parameter("side_speed_multiplier", 0.9)
        self.declare_parameter("forward_speed_multiplier", 0.4)
        self.declare_parameter("forward_speed", 0.18)
        self.declare_parameter("search_angular_speed", 0.5)
        self.declare_parameter("dist_thresh", 0.2)
        self.declare_parameter("filter_value", 0.9)

        self.angular_speed_multiplier = self.get_parameter('angular_speed_multiplier').get_parameter_value().double_value
        self.side_speed_multiplier = self.get_parameter('side_speed_multiplier').get_parameter_value().double_value
        self.forward_speed_multiplier = self.get_parameter('forward_speed_multiplier').get_parameter_value().double_value
        self.forward_speed = self.get_parameter('forward_speed').get_parameter_value().double_value
        self.search_angular_speed = self.get_parameter('search_angular_speed').get_parameter_value().double_value
        self.dist_thresh = self.get_parameter('dist_thresh').get_parameter_value().double_value
        self.filter_value = self.get_parameter('filter_value').get_parameter_value().double_value

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.target_val = 0.0
        self.target_dist = 0.0
        self.target_ang = 0.0

    def listener(self, data):
        filter_val = self.filter_value
        self.target_val = self.target_val * filter_val + data.x *(1-filter_val) # x-value representing the position of the marker in the picture frame
        self.target_dist = self.target_dist * filter_val + data.y * (1-filter_val)# y-value representing the distance to the marker
        self.target_ang = self.target_ang * filter_val + data.z * (1-filter_val)# z-value representing the angle of the marker to the robot

    def timer_callback(self):
        msg = Twist()
        self.get_logger().info(f"Val: {self.target_val},Ang: {self.target_ang}")
        if (self.target_dist > self.dist_thresh):
            msg.linear.x = self.forward_speed
        msg.linear.y =  -self.side_speed_multiplier * self.target_val
        msg.angular.z = -self.angular_speed_multiplier * self.target_ang
        self.get_logger().info(f"Vel_x: {msg.linear.x}, Vel_y: {msg.linear.y}, Ang_vel: {msg.angular.z}")
        self.pub_.publish(msg)