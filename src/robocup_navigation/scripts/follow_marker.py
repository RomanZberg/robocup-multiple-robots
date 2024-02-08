#!/usr/bin/env python3

import rclpy
from robocup_navigation.follow_marker_mod import FollowMarker


def main(args=None):

    rclpy.init(args=args)
    node = FollowMarker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()