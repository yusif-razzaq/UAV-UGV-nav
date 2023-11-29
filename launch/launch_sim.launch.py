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

    rsp_uav = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp_uav.launch.py'
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
    spawn_robot = Node(package='gazebo_ros', executable='spawn_entity.py', 
                        name='spawn_robot',
                        arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
                        output='screen')
    
    spawn_uav = Node(package='gazebo_ros', executable='spawn_entity.py', 
                    name='spawn_uav',
                    arguments=['-topic', 'robot_description', '-entity', 'my_uav'],
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
        rsp_uav,
        # spawn_robot,
        spawn_uav,
        gazebo,
        # joystick,
        # twist_mux,
        waypoints_server_node,
        waypoints_client_node,
        nav2_handler_node,
        # waypoint_publisher,
    ])
