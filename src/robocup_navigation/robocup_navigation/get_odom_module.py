#!/usr/bin/env python3

import requests
import json
from rclpy.node import Node
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation

class Odom(Node):
    URL = "http://127.0.0.1/data/odometry"
    PARAMS = {'sid':'robotino_rest_node'} 

    def __init__(self,myargv):
        super().__init__("odom")
        self.replaceURL(myargv)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.get_odom_data()
        self.timer = self.create_timer(0.5, self.get_odom_data)
        self.get_logger().info("Odom Publisher has been started")
    
    def replaceURL(self, myargv):
        if len(myargv)>1:
            Odom.URL = Odom.URL.replace("127.0.0.1",myargv[1])
            self.get_logger().info("URL")
        self.get_logger().info("connecting to: " + Odom.URL)

    def get_odom_data(self):
        try:
            msg = Odometry()
            r = requests.get(url = Odom.URL)
            data_odom = json.loads(r.text)
            
            msg.header.frame_id = "odom"
            msg.child_frame_id = "base_footprint"
            msg.header.stamp = self.get_clock().now().to_msg()
            
            msg.pose.pose.position.x = float(data_odom[0])
            msg.pose.pose.position.y = float(data_odom[1])
            
            quaternion = Odom.euler2quaternion(data_odom[2])
            
            msg.pose.pose.orientation.z = float(quaternion[2])
            msg.pose.pose.orientation.w = float(quaternion[3])

            msg.twist.twist.linear.x = float(data_odom[3])
            msg.twist.twist.linear.y = float(data_odom[4])
            msg.twist.twist.angular.z = float(data_odom[5])

            self.odom_pub.publish(msg)
            if r.status_code != requests.codes.ok:
                self.get_logger().warning("get from %s with params %s failed" %  (Odom.URL, Odom.PARAMS))
        except requests.exceptions.RequestException as e:
            self.get_logger().error("%s" % (e))
            pass

        

    def euler2quaternion(gamma):
        rot = Rotation.from_euler('xyz', [0, 0, gamma],degrees= False)

        rot_quat = rot.as_quat()
        return rot_quat