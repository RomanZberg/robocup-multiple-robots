#!/usr/bin/env python3

import rclpy
from robocup_navigation.flip_laser_module import FlipLaser

def main(args=None):
    rclpy.init(args=args)
    node = FlipLaser()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()