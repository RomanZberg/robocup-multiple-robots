#!/usr/bin/env python3

import rclpy
import requests
import json
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Omnidrive(Node):
    URL = "http://127.0.0.1/data/omnidrive"
    PARAMS = {'sid':'robotino_rest_node'} 

    def __init__(self,myargv):
        super().__init__("omnidrive")
        self.replaceURL(myargv)
        self.omnidrive_subscriber_ = self.create_subscription(Twist, "/cmd_vel",self.omnidrive_callback, 10)
        self.get_logger().info("Omnidrive Subscriber has been started")
    
    def replaceURL(self, myargv):
        if len(myargv)>1:
            Omnidrive.URL = Omnidrive.URL.replace("127.0.0.1",myargv[1])
            self.get_logger().info("URL")
        self.get_logger().info("connecting to: " + Omnidrive.URL)

    def omnidrive_callback(self, data: Twist):
        self.get_logger().info("%f %f %f" % (data.linear.x, data.linear.y, data.angular.z))
        pdata = [data.linear.x, data.linear.y, data.angular.z]
        print(pdata)
        try:
            r = requests.post(url = Omnidrive.URL, data = json.dumps(pdata) )
            self.get_logger().info(str(r.status_code))
            if r.status_code != requests.codes.ok:
                self.get_logger().warning("post to %s with params %s failed" %  (Omnidrive.URL, Omnidrive.PARAMS))
        except requests.exceptions.RequestException as e:
            self.get_logger().error("%s" % (e))
            pass