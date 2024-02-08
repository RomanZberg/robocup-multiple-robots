#!/usr/bin/python3
import os
import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    ld = LaunchDescription()

    pkg_gazebo_ros = FindPackageShare(package="gazebo_ros").find("gazebo_ros")
    pkg_share = FindPackageShare(package="robocup_navigation").find("robocup_navigation")

    default_rviz_config_path = os.path.join(pkg_share, 'rviz/rviz_basic_settings.rviz')
    
    world_file = "rcll-2017-default.world"
    world_path = os.path.join(pkg_share, "rcll_sim", "worlds", world_file)

    gazebo_models_path = os.path.join(pkg_share, "rcll_sim", "meshes")
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    #Robot description
    robot_file = "robotino.urdf"
    robot_path = os.path.join(pkg_share, "robotino3_description", "robotino", "RobotinoModel", "urdf", robot_file)
    robot_xacro = xacro.process_file(robot_path).toxml()

    # launch Gazebo by including its definition
    gazebo = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')),
      launch_arguments={'world': world_path}.items()
   )
    
    robot_state_pubilsher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_xacro,
                    'use_sim_time': True}] # add other parameters here if required
   )

    spawn_entity = Node(
      package='gazebo_ros',
      executable='spawn_entity.py',
      arguments=['-topic', 'robot_description',
                 '-entity', 'robotino',
                 '-x','4.5',
                 '-y','0.5',
                 '-Y','3.14'],
      output='screen'
    )
    
    ld.add_action(gazebo)
    ld.add_action(robot_state_pubilsher)
    ld.add_action(spawn_entity)

    return ld