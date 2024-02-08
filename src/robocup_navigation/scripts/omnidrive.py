#!/usr/bin/env python3

import rclpy
import json
from geometry_msgs.msg import Twist
import sys
from robocup_navigation.omnidrive_module import Omnidrive

def main(args=None):
    rclpy.init(args=args)
    myargv = sys.argv
    node = Omnidrive(myargv)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()