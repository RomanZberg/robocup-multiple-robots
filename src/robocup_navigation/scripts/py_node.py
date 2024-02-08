#!/usr/bin/env python3

import rclpy
from robocup_navigation.circle_module import MyNode

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()