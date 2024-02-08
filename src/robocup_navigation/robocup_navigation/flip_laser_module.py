#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

class FlipLaser(Node):
    def __init__(self):
        super().__init__('flip_laser')

        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE
        

        self.laser_sub = self.create_subscription(LaserScan,"/lidar_3/scan",self.callback,qos_profile)
        self.laser_pub = self.create_publisher(LaserScan, "/lidar_1/scan", 10)
        self.get_logger().info("Laser Flipper started")

    def callback(self,data):
        msg = LaserScan()

        msg.header = data.header
        msg.angle_max = data.angle_max
        msg.angle_min = data.angle_min
        msg.angle_increment = data.angle_increment
        msg.time_increment = data.time_increment
        msg.scan_time = data.scan_time
        msg.range_min = data.range_min
        msg.range_max = data.range_max

        msg.ranges = FlipLaser.reverse(data.ranges)

        msg.intensities = FlipLaser.reverse(data.intensities)
        self.laser_pub.publish(msg)
    
    def reverse(list):
        list.reverse()
        return list