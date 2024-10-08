# Copyright 2023 driveblocks GmbH
# driveblocks proprietary license
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # novatel to pose converter
    novatel_to_pose =  Node(
        package='novatel_ros_converter',
        executable='novatel_to_pose',
        name='novatel_to_pose',
        namespace='novatel_to_pose',
        remappings=[
            ('input/pos', '/novatel/oem7/bestpos'),
            ('input/vel', '/novatel/oem7/bestvel'),
        ]
    )

    return LaunchDescription([
        novatel_to_pose,
    ])
