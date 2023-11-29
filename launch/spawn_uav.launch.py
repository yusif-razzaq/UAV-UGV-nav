import os
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = os.path.join(get_package_share_directory('my_bot'))

    xacro_file_uav = os.path.join(pkg_path,'description','uav.urdf.xacro')
    uav_description_config = Command(['xacro ', xacro_file_uav])

    params = {'robot_description': uav_description_config}
    node_uav_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    return LaunchDescription([
        # Start the robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher'
        ),
        # Spawn the robot using spawn_entity.py
        ExecuteProcess(
            cmd=[f'ros2 run robot_state_publisher spawn_entity.py urdf_file:={xacro_file_uav}'],
            output='screen'
        )
    ])
