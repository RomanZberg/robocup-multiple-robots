import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
   
   pkg_share = FindPackageShare(package="robocup_navigation").find("robocup_navigation") 
   pkg = "robocup_navigation"
   ld = LaunchDescription()
      
   camera = Node(
      package="usb_cam",
      executable="usb_cam_node_exe",
      name="usb_cam",
      arguments=[]
   )

   omnidrive = Node(
      package= pkg,
      executable="omnidrive.py",
      name="omidrive",
      arguments=["192.168.1.167"]
   )

   detect_marker = Node(
      package= pkg,
      executable="detect_marker.py",
      name="detect_marker",
      remappings=[
         ("image_in", "/image_raw")
         ]
   )

   ld.add_action(camera)
   ld.add_action(omnidrive)
   ld.add_action(detect_marker)
   return ld
