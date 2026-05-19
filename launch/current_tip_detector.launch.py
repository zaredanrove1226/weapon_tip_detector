#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('weapon_tip_detector')

    config_file = os.path.join(
        pkg_share,
        'config',
        'current_tip_detector.yaml'
    )

    current_slot_id = LaunchConfiguration('current_slot_id')

    declare_current_slot_id = DeclareLaunchArgument(
        'current_slot_id',
        default_value='4',
        description='Current weapon tip slot id. Valid range: 1~6.'
    )

    current_tip_detector_node = Node(
        package='weapon_tip_detector',
        executable='current_tip_detector_node_refactored',
        name='current_tip_detector_node',
        output='screen',
        parameters=[
            config_file,
            {
                'current_slot_id': current_slot_id,
            }
        ],
    )

    return LaunchDescription([
        declare_current_slot_id,
        current_tip_detector_node,
    ])