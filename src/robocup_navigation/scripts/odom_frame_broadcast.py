#!/usr/bin/env python3
from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry


class FixedFrameBroadcaster(Node):

   def __init__(self):
       super().__init__('fixed_frame_tf2_broadcaster')
       self.odom_sub = self.create_subscription(Odometry,'/odometry/filtered',self.odom_callback,10)
       self.tf_broadcaster = TransformBroadcaster(self)
       self.timer = self.create_timer(0.1, self.broadcast_timer_callback)

       self.odom = Odometry()
       

   def odom_callback(self,data):
       self.odom.pose.pose.position.x = data.pose.pose.position.x
       self.odom.pose.pose.position.y = data.pose.pose.position.y
       self.odom.pose.pose.position.z = data.pose.pose.position.z
       self.odom.pose.pose.orientation.x = data.pose.pose.orientation.x
       self.odom.pose.pose.orientation.y = data.pose.pose.orientation.y
       self.odom.pose.pose.orientation.z = data.pose.pose.orientation.z
       self.odom.pose.pose.orientation.w = data.pose.pose.orientation.w
       self.get_logger().info("%f " % self.odom.pose.pose.position.x)
   
   def broadcast_timer_callback(self):
       t = TransformStamped()

       t.header.stamp = self.get_clock().now().to_msg()
       t.header.frame_id = 'odom'
       t.child_frame_id = 'base_footprint'
       t.transform.translation.x = self.odom.pose.pose.position.x
       t.transform.translation.y = self.odom.pose.pose.position.y
       t.transform.translation.z = self.odom.pose.pose.position.z
       t.transform.rotation.x = self.odom.pose.pose.orientation.x
       t.transform.rotation.y = self.odom.pose.pose.orientation.y
       t.transform.rotation.z = self.odom.pose.pose.orientation.z
       t.transform.rotation.w = self.odom.pose.pose.orientation.w

       self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = FixedFrameBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == "__main__":
    main()