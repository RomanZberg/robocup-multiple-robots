#!/usr/bin/env python3

import rclpy
from robocup_navigation.detect_marker_module import DetectMarker
import robocup_navigation.process_image as proc

def main(args=None):

    rclpy.init(args=args)
    node = DetectMarker()
    while rclpy.ok():
        rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()