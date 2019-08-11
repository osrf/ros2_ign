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

#ifndef ROS2_IGN_BRIDGE__BUILTIN_INTERFACES_FACTORIES_HPP_
#define ROS2_IGN_BRIDGE__BUILTIN_INTERFACES_FACTORIES_HPP_

// include ROS 2 messages
#include <std_msgs/msg/string.hpp>

// include Ignition Transport messages
#include <ignition/msgs.hh>

#include <ros2_ign_bridge/factory.hpp>

#include <memory>
#include <string>

namespace ros2_ign_bridge
{

std::shared_ptr<FactoryInterface>
get_factory_builtin_interfaces(
  const std::string & ros2_type_name,
  const std::string & ign_type_name);

std::shared_ptr<FactoryInterface>
get_factory(
  const std::string & ros2_type_name,
  const std::string & ign_type_name);

// conversion functions for available interfaces

// std_msgs
template<>
void
Factory<
  std_msgs::msg::String,
  ignition::msgs::StringMsg
>::convert_ros2_to_ign(
  const std_msgs::msg::String & ros2_msg,
  ignition::msgs::StringMsg & ign_msg);

template<>
void
Factory<
  std_msgs::msg::String,
  ignition::msgs::StringMsg
>::convert_ign_to_ros2(
  const ignition::msgs::StringMsg & ign_msg,
  std_msgs::msg::String & ros2_msg);

}  // namespace ros2_ign_bridge

#endif  // ROS2_IGN_BRIDGE__BUILTIN_INTERFACES_FACTORIES_HPP_
