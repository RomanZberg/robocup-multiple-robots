#!/usr/bin/env python3

import rclpy
from robocup_navigation.camera_module import Camera

def main(args=None):
    rclpy.init(args=args)
    node = Camera()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()