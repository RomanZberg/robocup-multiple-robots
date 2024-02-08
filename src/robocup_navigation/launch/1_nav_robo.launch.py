import os
import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
   
   pkg_share = FindPackageShare(package="robocup_navigation").find("robocup_navigation") 
    
   ld = LaunchDescription()
   
   rviz2_config_file = "test.rviz"
   rviz2_config_path = os.path.join(pkg_share, "rcll_sim", "rviz", rviz2_config_file)
   rviz2 = Node(
      package="rviz2",
      executable="rviz2",
      name="rviz2",
      arguments=["-d" , rviz2_config_path]
   )
   
   pkg_slam = FindPackageShare(package="slam_toolbox").find("slam_toolbox")
   slam_param_file = "mapper_params_online_async.yaml"
   slam_param = os.path.join(pkg_share, "config", slam_param_file)
   slam = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'online_async_launch.py')),
      launch_arguments={'slam_params_file': slam_param,
                        'use_sim_time': 'true'}.items()
   )

   nav2 = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'navigation_launch.py')),
      launch_arguments={'use_sim_time': 'true'}.items()
   )

   sim_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(os.path.join(pkg_share, 'rcll_sim', 'launch', 'sim_launch.launch.py'))
   )

   ld.add_action(nav2)
   ld.add_action(sim_launch)
   ld.add_action(rviz2)
   ld.add_action(slam)
   return ld
