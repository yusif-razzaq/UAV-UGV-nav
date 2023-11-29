import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('my_bot'))
    xacro_file_uav = os.path.join(pkg_path,'description','uav.urdf.xacro')
    uav_description_config = Command(['xacro ', xacro_file_uav])

    params = {'robot_description': uav_description_config}
    node_uav_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='uav_state_publisher',
        output='screen',
        parameters=[params]
    )
    
    # Launch!
    return LaunchDescription([
        node_uav_state_publisher
    ])
