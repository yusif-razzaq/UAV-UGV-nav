import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='my_bot'

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'false'}.items()
    )

    gazebo_params_path = os.path.join(get_package_share_directory(package_name), 'config', 'gazebo_params.yaml')

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_path}.items()
            )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
                        output='screen')
    
    joystick = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory(package_name),'launch','joystick.launch.py'
            )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/cmd_vel')]
        )
    
    xacro_file_uav = os.path.join(get_package_share_directory(package_name),'description','robot.urdf.xacro')
    print(xacro_file_uav)
    uav_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_2',
        # Pass the URDF file path for robot 2
        arguments=[xacro_file_uav]
    )

    # waypoint_publisher = Node(
    #         package='my_bot',
    #         executable=exec_waypoints,
    #         output='screen',
    #     )

    # waypoints_pub_path = os.path.join(get_package_share_directory(package_name),'src','waypoints_publisher.py')
    # waypoint_publisher = ExecuteProcess(
    #     cmd=['python3', '-u', waypoints_pub_path],  # Command to run the script
    #     output='screen',
    # )

    # waypoints_sub_path = os.path.join(get_package_share_directory(package_name),'src','waypoints_subscriber.py')    
    # waypoint_subscriber = ExecuteProcess(
    #     cmd=['python3', '-u', waypoints_sub_path],  # Command to run the script
    #     output='screen',
    # )

    # spawn_warehouse_bot = Node(package='warehouse_robot_spawner_pkg', executable='spawn_demo',
    #                     arguments=['WarehouseBot', 'demo', '-1.5', '-4.0', '0.0'],
    #                     output='screen')

    # diff_drive_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["diff_cont"],
    # )

    # joint_broad_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["joint_broad"],
    # )

    waypoints_server_node = Node(
        package='my_bot',  # Replace with your package name
        executable='waypoints_server',  # Name of your waypoints_server executable
        output='screen',
    )

    # Add waypoints_client node
    waypoints_client_node = Node(
        package='my_bot',  # Replace with your package name
        executable='waypoints_client',  # Name of your waypoints_client executable
        output='screen',
    )

    nav2_handler_node = Node(
        package='my_bot',  # Replace with your package name
        executable='nav2_handler',  # Name of your waypoints_server executable
        output='screen',
    )
    
    # Launch them all!
    return LaunchDescription([
        # rsp,
        # gazebo,
        # spawn_entity,
        # joystick,
        # twist_mux,
        # waypoints_server_node,
        # waypoints_client_node,
        # nav2_handler_node,
        uav_state_publisher
        # waypoint_publisher,
        # spawn_warehouse_bot,
        # diff_drive_spawner,
        # joint_broad_spawner
    ])
