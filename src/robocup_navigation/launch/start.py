import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo, RegisterEventHandler, TimerAction


def generate_launch_description():
    # Constants for paths to different files and folders

    pkg_share = FindPackageShare(package="robocup_navigation").find("robocup_navigation")

    package_name = 'robocup_navigation'

    world_file = "rcll-2017-default.world"
    world_path = os.path.join(pkg_share, "rcll_sim", "worlds", world_file)

    gazebo_models_path = os.path.join(pkg_share, "rcll_sim", "meshes")
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    sdf_model_path_1 = 'robotino3_description/robotino3_hslu_1/model.sdf'
    sdf_model_path_2 = 'robotino3_description/robotino3_hslu_2/model.sdf'
    sdf_model_path_3 = 'robotino3_description/robotino3_hslu_3/model.sdf'

    # Pose where we want to spawn the robot
    spawn_x_val = '0.0'
    spawn_y_val = '0.0'
    spawn_z_val = '0.0'
    spawn_yaw_val = '0.0'

    ############ You do not need to change anything below this line #############

    # Set the path to different files and folders.
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
    pkg_share = FindPackageShare(package=package_name).find(package_name)

    sdf_model_path_1 = os.path.join(pkg_share, sdf_model_path_1)
    sdf_model_path_2 = os.path.join(pkg_share, sdf_model_path_2)
    sdf_model_path_3 = os.path.join(pkg_share, sdf_model_path_3)

    # Launch configuration variables specific to simulation

    headless = LaunchConfiguration('headless')
    use_simulator = LaunchConfiguration('use_simulator')

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        name='use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_simulator_cmd = DeclareLaunchArgument(
        name='headless',
        default_value='False',
        description='Whether to execute gzclient')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_use_simulator_cmd = DeclareLaunchArgument(
        name='use_simulator',
        default_value='True',
        description='Whether to start the simulator')

    # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        condition=IfCondition(use_simulator),
        launch_arguments={'world': world_path}.items()
    )

    # Start Gazebo client
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])),
        launch_arguments={'world': world_path}.items()
    )

    # Launch the robot
    spawn_entity_cmd_1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        namespace='robotino1',
        arguments=['-entity', 'robotino1',
                   '-file', sdf_model_path_1,
                   '-x', '4.5',
                   '-y', '0.5',
                   '-z', spawn_z_val,
                   '-Y', spawn_yaw_val],
        output='screen')

    spawn_entity_cmd_2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        namespace='robotino2',
        arguments=['-entity', 'robotino2',
                   '-file', sdf_model_path_2,
                   '-x', '5.5',
                   '-y', '0.5',
                   '-z', spawn_z_val,
                   '-Y', spawn_yaw_val],
        output='screen')

    spawn_entity_cmd_3 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        namespace='robotino3',
        arguments=['-entity', 'robotino3',
                   '-file', sdf_model_path_3,
                   '-x', '6.5',
                   '-y', '0.5',
                   '-z', spawn_z_val,
                   '-Y', spawn_yaw_val],
        output='screen')

    rviz2_config_file = "test.rviz"
    rviz2_config_path = os.path.join(pkg_share, "rcll_sim", "rviz", rviz2_config_file)
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz2_config_path]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_simulator_cmd)

    # Add any actions
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(spawn_entity_cmd_1)
    # ld.add_action(spawn_entity_cmd_2)
    # ld.add_action(spawn_entity_cmd_3)

    ld.add_action(rviz2)

    # ld.add_action(
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource([
    #             FindPackageShare("slam_toolbox"), '/launch', '/online_async_launch.py']),
    #         launch_arguments={
    #             'slam_params_file': [
    #                 FindPackageShare("robocup_navigation"), '/config', '/mapper_params_online_async_robotino_2.yaml']
    #
    #         }.items(),
    #     )
    # )
    #
    # ld.add_action(
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource([
    #             FindPackageShare("slam_toolbox"), '/launch', '/online_async_launch.py']),
    #         launch_arguments={
    #             'slam_params_file': [
    #                 FindPackageShare("robocup_navigation"), '/config', '/mapper_params_online_async_robotino_3.yaml']
    #
    #         }.items(),
    #     )
    # )

    static_transform_publisher_node_1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0.05', '0.3', '0.0', '0', '0', '0', 'robotino3_hslu_1/body', 'robotino3_hslu_1/laser']
    )

    # static_transform_publisher_node_2 = Node(
    #    package='tf2_ros',
    #    executable='static_transform_publisher',
    #    name='static_transform_publisher',
    #    arguments=['0.05', '0.3', '0.0', '0', '0', '0', 'robotino3_hslu_2/body', 'robotino3_hslu_2/laser']
    # )

    # static_transform_publisher_node_3 = Node(
    #    package='tf2_ros',
    #    executable='static_transform_publisher',
    #    name='static_transform_publisher',
    #    arguments=['0.05', '0.3', '0.0', '0', '0', '0', 'robotino3_hslu_3/body', 'robotino3_hslu_3/laser']
    # )


    ld.add_action(static_transform_publisher_node_1)
    # ld.add_action(static_transform_publisher_node_2)
    # ld.add_action(static_transform_publisher_node_3)

    # Define the path to your custom parameters file
    params_file_path = os.path.join(pkg_share, 'config', 'nav2_params_sim.yaml')

    ld.add_action(
        TimerAction(
            period=3.0,  # This should be large enough that Gazebo is loaded before SLAM is started
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        FindPackageShare("slam_toolbox"), '/launch', '/online_async_launch.py']),
                    launch_arguments={
                        "use_sim_time": "true",
                        'slam_params_file': [
                            FindPackageShare("robocup_navigation"), '/config',
                            '/mapper_params_online_async_robotino_1.yaml']

                    }.items(),
                )
            ]
        ),

    )

    ld.add_action(
        TimerAction(
            period=6.0,  # This sould be large enough that SLAM is fully started before the rest of nav2 is launched
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        FindPackageShare("robocup_navigation"), '/launch', '/navigation.py']),
                    launch_arguments={
                        "use_sim_time": "true",
                        'params_file': params_file_path
                    }.items(),
                )
            ]
        ),
    )

    return ld
