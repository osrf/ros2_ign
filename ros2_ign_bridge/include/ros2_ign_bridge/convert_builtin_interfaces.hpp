// Copyright 2019 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROS2_IGN_BRIDGE__CONVERT_BUILTIN_INTERFACES_HPP_
#define ROS2_IGN_BRIDGE__CONVERT_BUILTIN_INTERFACES_HPP_

#include <rclcpp/time.hpp>

// include ROS 2 builtin messages
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

// include Ignition builtin messages
#include <ignition/msgs.hh>

#include <ros2_ign_bridge/convert_decl.hpp>

namespace ros2_ign_bridge
{

// std_msgs
template<>
void
convert_ros2_to_ign(
  const std_msgs::msg::Float32 & ros2_msg,
  ignition::msgs::Float & ign_msg);

template<>
void
convert_ign_to_ros2(
  const ignition::msgs::Float & ign_msg,
  std_msgs::msg::Float32 & ros2_msg);

template<>
void
convert_ros2_to_ign(
  const std_msgs::msg::Header & ros2_msg,
  ignition::msgs::Header & ign_msg);

template<>
void
convert_ign_to_ros2(
  const ignition::msgs::Header & ign_msg,
  std_msgs::msg::Header & ros2_msg);

template<>
void
convert_ros2_to_ign(
  const std_msgs::msg::String & ros2_msg,
  ignition::msgs::StringMsg & ign_msg);

template<>
void
convert_ign_to_ros2(
  const ignition::msgs::StringMsg & ign_msg,
  std_msgs::msg::String & ros2_msg);

// rosgraph_msgs
template<>
void
convert_ign_to_ros2(
  const ignition::msgs::Clock & ign_msg,
  rosgraph_msgs::msg::Clock & ros2_msg);

template<>
void
convert_ros2_to_ign(
  const rosgraph_msgs::msg::Clock & ros2_msg,
  ignition::msgs::Clock & ign_msg);

// geometry_msgs
template<>
void
convert_ros2_to_ign(
  const geometry_msgs::msg::Quaternion & ros2_msg,
  ignition::msgs::Quaternion & ign_msg);

template<>
void
convert_ign_to_ros2(
  const ignition::msgs::Quaternion & ign_msg,
  geometry_msgs::msg::Quaternion & ros2_msg);

template<>
void
convert_ros2_to_ign(
  const geometry_msgs::msg::Vector3 & ros2_msg,
  ignition::msgs::Vector3d & ign_msg);

template<>
void
convert_ign_to_ros2(
  const ignition::msgs::Vector3d & ign_msg,
  geometry_msgs::msg::Vector3 & ros2_msg);

template<>
void
convert_ros2_to_ign(
  const geometry_msgs::msg::Point & ros2_msg,
  ignition::msgs::Vector3d & ign_msg);

template<>
void
convert_ign_to_ros2(
  const ignition::msgs::Vector3d & ign_msg,
  geometry_msgs::msg::Point & ros2_msg);

template<>
void
convert_ros2_to_ign(
  const geometry_msgs::msg::Pose & ros2_msg,
  ignition::msgs::Pose & ign_msg);

template<>
void
convert_ign_to_ros2(
  const ignition::msgs::Pose & ign_msg,
  geometry_msgs::msg::Pose & ros2_msg);

template<>
void
convert_ros2_to_ign(
  const geometry_msgs::msg::PoseStamped & ros2_msg,
  ignition::msgs::Pose & ign_msg);

template<>
void
convert_ign_to_ros2(
  const ignition::msgs::Pose & ign_msg,
  geometry_msgs::msg::PoseStamped & ros2_msg);

template<>
void
convert_ros2_to_ign(
  const geometry_msgs::msg::Transform & ros2_msg,
  ignition::msgs::Pose & ign_msg);

template<>
void
convert_ign_to_ros2(
  const ignition::msgs::Pose & ign_msg,
  geometry_msgs::msg::Transform & ros2_msg);

template<>
void
convert_ros2_to_ign(
  const geometry_msgs::msg::TransformStamped & ros2_msg,
  ignition::msgs::Pose & ign_msg);

template<>
void
convert_ign_to_ros2(
  const ignition::msgs::Pose & ign_msg,
  geometry_msgs::msg::TransformStamped & ros2_msg);

template<>
void
convert_ros2_to_ign(
  const geometry_msgs::msg::Twist & ros2_msg,
  ignition::msgs::Twist & ign_msg);

template<>
void
convert_ign_to_ros2(
  const ignition::msgs::Twist & ign_msg,
  geometry_msgs::msg::Twist & ros2_msg);

// nav_msgs
template<>
void
convert_ros2_to_ign(
  const nav_msgs::msg::Odometry & ros2_msg,
  ignition::msgs::Odometry & ign_msg);

template<>
void
convert_ign_to_ros2(
  const ignition::msgs::Odometry & ign_msg,
  nav_msgs::msg::Odometry & ros2_msg);

// sensor_msgs
template<>
void
convert_ros2_to_ign(
  const sensor_msgs::msg::FluidPressure & ros2_msg,
  ignition::msgs::FluidPressure & ign_msg);

template<>
void
convert_ign_to_ros2(
  const ignition::msgs::FluidPressure & ign_msg,
  sensor_msgs::msg::FluidPressure & ros2_msg);

template<>
void
convert_ros2_to_ign(
  const sensor_msgs::msg::Image & ros2_msg,
  ignition::msgs::Image & ign_msg);

template<>
void
convert_ign_to_ros2(
  const ignition::msgs::Image & ign_msg,
  sensor_msgs::msg::Image & ros2_msg);

template<>
void
convert_ros2_to_ign(
  const sensor_msgs::msg::CameraInfo & ros2_msg,
  ignition::msgs::CameraInfo & ign_msg);

template<>
void
convert_ign_to_ros2(
  const ignition::msgs::CameraInfo & ign_msg,
  sensor_msgs::msg::CameraInfo & ros2_msg);

template<>
void
convert_ros2_to_ign(
  const sensor_msgs::msg::Imu & ros2_msg,
  ignition::msgs::IMU & ign_msg);

template<>
void
convert_ign_to_ros2(
  const ignition::msgs::IMU & ign_msg,
  sensor_msgs::msg::Imu & ros2_msg);

template<>
void
convert_ros2_to_ign(
  const sensor_msgs::msg::JointState & ros2_msg,
  ignition::msgs::Model & ign_msg);

template<>
void
convert_ign_to_ros2(
  const ignition::msgs::Model & ign_msg,
  sensor_msgs::msg::JointState & ros2_msg);

template<>
void
convert_ros2_to_ign(
  const sensor_msgs::msg::LaserScan & ros2_msg,
  ignition::msgs::LaserScan & ign_msg);

template<>
void
convert_ign_to_ros2(
  const ignition::msgs::LaserScan & ign_msg,
  sensor_msgs::msg::LaserScan & ros2_msg);

template<>
void
convert_ros2_to_ign(
  const sensor_msgs::msg::MagneticField & ros2_msg,
  ignition::msgs::Magnetometer & ign_msg);

template<>
void
convert_ign_to_ros2(
  const ignition::msgs::Magnetometer & ign_msg,
  sensor_msgs::msg::MagneticField & ros2_msg);

template<>
void
convert_ros2_to_ign(
  const sensor_msgs::msg::PointCloud2 & ros2_msg,
  ignition::msgs::PointCloudPacked & ign_msg);

template<>
void
convert_ign_to_ros2(
  const ignition::msgs::PointCloudPacked & ign_msg,
  sensor_msgs::msg::PointCloud2 & ros2_msg);

template<>
void
convert_ros2_to_ign(
  const sensor_msgs::msg::BatteryState & ros2_msg,
  ignition::msgs::BatteryState & ign_msg);

template<>
void
convert_ign_to_ros2(
  const ignition::msgs::BatteryState & ign_msg,
  sensor_msgs::msg::BatteryState & ros2_msg);

}  // namespace ros2_ign_bridge

#endif  // ROS2_IGN_BRIDGE__CONVERT_BUILTIN_INTERFACES_HPP_
