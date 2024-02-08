import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource



from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'robocup_navigation'
    file_subpath = 'robotino3_description/robotino/RobotinoModel/urdf/robotino.urdf'
    world_subpath = 'rcll_sim/worlds/rcll-2017-default.world'


    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    world_prefix = os.path.join(get_package_share_directory(pkg_name), world_subpath)


    # Configure the node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': True}] # add other parameters here if required
    )

    gazebo_params_file = os.path.join(get_package_share_directory(pkg_name),'config','gazebo_params.yaml')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
            #launch_arguments = {'world': world_prefix}.items()
            launch_arguments = {'extra_gazebo_args': '--ros-args --params-file '+ gazebo_params_file}.items()
        )
    
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'robotino'],
                    output='screen')
    
    # Run the node
    return LaunchDescription([
        node_robot_state_publisher,
        gazebo,
        spawn_entity
    ])


