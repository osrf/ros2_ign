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

"""Launch rqt_plot and Ignition Gazebo with battery world."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/ign_gazebo.launch.py']),
            launch_arguments={'args': '-r -v 3 linear_battery_demo.sdf'}.items(),
        ),

        Node(
            package='ros2_ign_bridge', node_executable='parameter_bridge',
            arguments=['/model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                       '/model/vehicle_blue/battery/linear_battery/state@'
                       'sensor_msgs/msg/BatteryState@ignition.msgs.BatteryState',
                       ],
            output='screen',
        ),

        Node(
            package='rqt_plot', node_executable='rqt_plot',
            arguments=['/model/vehicle_blue/battery/linear_battery/state/percentage'],
        ),
    ])
