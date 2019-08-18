# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch RViz and Ignition Gazebo with lidar sensor world."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    rviz_path = os.path.join(get_package_share_directory('ros2_ign_gazebo_demos'),
                             'rviz', 'gpu_lidar_bridge.rviz')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/ign_gazebo.launch.py']),
            launch_arguments={'args': '-r -v 3 gpu_lidar_sensor.sdf'}.items(),
        ),

        Node(
            package='ros2_ign_bridge', node_executable='parameter_bridge',
            arguments=['/lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
                       '/lidar/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
                       ],
            output='screen',
        ),

        Node(
            package='rviz2', node_executable='rviz2',
            arguments=['-d', rviz_path],
        ),
    ])
