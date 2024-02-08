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

   remappings = [('/tf', 'tf'),
                ('/tf_static', 'tf_static')]
   
   rviz2_config_file = "navigation.rviz"
   rviz2_config_path = os.path.join(pkg_share, "rcll_sim", "rviz", rviz2_config_file)
   rviz2 = Node(
      package="rviz2",
      executable="rviz2",
      name="rviz2",
      arguments=["-d" , rviz2_config_path]
   )
   
   pkg_slam = FindPackageShare(package="slam_toolbox").find("slam_toolbox")
   slam_param_file = "mapper_params_online_async_robotino.yaml"
   slam_param = os.path.join(pkg_share, "config", slam_param_file)
   slam = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'online_async_launch.py')),
      launch_arguments={'slam_params_file': slam_param,
                        'use_sim_time': 'false'}.items()
   )

   nav2 = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'navigation_launch.py')),
      launch_arguments={'use_sim_time': 'false'}.items()
   )
   
   pkg_sick_laser = FindPackageShare(package="sick_scan_xd").find("sick_scan_xd")
   tim_launch_file_path = os.path.join(pkg_sick_laser, 'launch/sick_tim_5xx.launch')
   laser_front = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        output='screen',
        arguments=[
            tim_launch_file_path,
            'nodename:=lidar_3',
            'hostname:=169.254.235.100',
            'cloud_topic:=lidar_3',
            'frame_id:=laser_link'
        ]
    )
   
   laser_back = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        output='screen',
        arguments=[
            tim_launch_file_path,
            'nodename:=lidar_2',
            'hostname:=169.254.235.110',
            'cloud_topic:=lidar_2',
            'frame_id:=laser_link'
        ]
    )

   pkg_laser_merger = FindPackageShare(package="ros2_laser_scan_merger").find("ros2_laser_scan_merger")
   laser_merger = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(os.path.join(pkg_laser_merger, 'launch', 'merge_2_scan.launch.py'))
   )

   omnidrive = Node(
      package= "robocup_navigation" ,
      executable="omnidrive.py",
      name="omidrive",
      arguments=["192.168.1.167"]
   )

   odom = Node(
      package= "robocup_navigation" ,
      executable="odom.py",
      name="odom",
      arguments=["192.168.1.167"]
   )

   flip_laser = Node(
      package= "robocup_navigation" ,
      executable="flip_laser.py",
      name="flip_laser"
   )
   odom_broadcaster = Node(
      package= "robocup_navigation" ,
      executable="odom_frame_broadcast.py",
      name="odom_broadcaster"
   )

   #Robot description
   robot_file = "robotino.urdf"
   robot_path = os.path.join(pkg_share, "robotino3_description", "robotino", "RobotinoModel", "urdf", robot_file)
   robot_xacro = xacro.process_file(robot_path).toxml()
   
   robot_state_pubilsher = Node(
      package='robot_state_publisher',
      executable='robot_state_publisher',
      output='screen',
      parameters=[{'robot_description': robot_xacro,
                  'use_sim_time': False}], # add other parameters here if required
      remappings = remappings
   )

   robot_localization_file_path = os.path.join(pkg_share, 'config/ekf.yaml')
   start_robot_localization_cmd = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    parameters=[robot_localization_file_path, 
    {'use_sim_time': False}])


   ld.add_action(nav2)
   ld.add_action(omnidrive)
   ld.add_action(odom)
   ld.add_action(flip_laser)
   ld.add_action(laser_front)
   ld.add_action(laser_back)
   ld.add_action(laser_merger)
   ld.add_action(robot_state_pubilsher)
   ld.add_action(start_robot_localization_cmd)
   ld.add_action(rviz2)
   ld.add_action(slam)
   return ld
