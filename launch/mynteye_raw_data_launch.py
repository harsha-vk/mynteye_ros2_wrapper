#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(get_package_share_directory('mynteye_ros2_wrapper'), 'config', 'params.yaml')  
    node=Node(
        package = 'mynteye_ros2_wrapper',
        name = 'mynteye_raw_data',
        executable = 'mynteye_raw_data',
        parameters = [config]
    )
    ld.add_action(node)
    return ld