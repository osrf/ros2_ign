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

#include <memory>
#include <string>

// include builtin interfaces
#include "ros2_ign_bridge/builtin_interfaces_factories.hpp"
#include "ros2_ign_bridge/convert_builtin_interfaces.hpp"

namespace ros2_ign_bridge
{

std::shared_ptr<FactoryInterface>
get_factory_builtin_interfaces(
  const std::string & ros2_type_name,
  const std::string & ign_type_name)
{
  // mapping from string to specialized template
  if (
    (ros2_type_name == "std_msgs/String" || ros2_type_name == "") &&
    ign_type_name == "ignition.msgs.StringMsg")
  {
    return std::make_shared<
      Factory<
        std_msgs::msg::String,
        ignition::msgs::StringMsg
      >
    >("std_msgs/String", ign_type_name);
  }
  return std::shared_ptr<FactoryInterface>();
}

std::shared_ptr<FactoryInterface>
get_factory(
  const std::string & ros2_type_name,
  const std::string & ign_type_name)
{
  std::shared_ptr<FactoryInterface> factory;
  factory = get_factory_builtin_interfaces(ros2_type_name, ign_type_name);
  if (factory) {
    return factory;
  }

  throw std::runtime_error("No template specialization for the pair");
}

// conversion functions for available interfaces

// std_msgs
template<>
void
Factory<
  std_msgs::msg::String,
  ignition::msgs::StringMsg
>::convert_ros2_to_ign(
  const std_msgs::msg::String & ros2_msg,
  ignition::msgs::StringMsg & ign_msg)
{
  ros2_ign_bridge::convert_ros2_to_ign(ros2_msg, ign_msg);
}

template<>
void
Factory<
  std_msgs::msg::String,
  ignition::msgs::StringMsg
>::convert_ign_to_ros2(
  const ignition::msgs::StringMsg & ign_msg,
  std_msgs::msg::String & ros2_msg)
{
  ros2_ign_bridge::convert_ign_to_ros2(ign_msg, ros2_msg);
}

}  // namespace ros2_ign_bridge
