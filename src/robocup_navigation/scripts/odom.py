#!/usr/bin/env python3

import rclpy
import sys
from robocup_navigation.get_odom_module import Odom

def main(args=None):
    rclpy.init(args=args)
    myargv = sys.argv
    node = Odom(myargv)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()