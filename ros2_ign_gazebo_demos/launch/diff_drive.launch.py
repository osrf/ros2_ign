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

"""Launch RViz and Ignition Gazebo with diff drive world."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    rviz_path = os.path.join(get_package_share_directory('ros2_ign_gazebo_demos'),
                             'rviz', 'diff_drive.rviz')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/ign_gazebo.launch.py']),
            launch_arguments={'args': '-r -v 3 diff_drive.sdf'}.items(),
        ),

        Node(
            package='ros2_ign_bridge', node_executable='parameter_bridge',
            arguments=['/model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                       '/model/vehicle_blue/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
                       '/model/vehicle_green/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                       '/model/vehicle_green/odometry@nav_msgs/msg/Odometry@'
                       'ignition.msgs.Odometry',
                       ],
            output='screen',
        ),

        Node(
            package='rviz2', node_executable='rviz2',
            arguments=['-d', rviz_path],
        ),
    ])
