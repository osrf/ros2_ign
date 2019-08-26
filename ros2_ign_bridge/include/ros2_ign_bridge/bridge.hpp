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

#ifndef ROS2_IGN_BRIDGE__BRIDGE_HPP_
#define ROS2_IGN_BRIDGE__BRIDGE_HPP_

// include Ignition Transport
#include <ignition/transport/Node.hh>

#include <ros2_ign_bridge/builtin_interfaces_factories.hpp>

#include <memory>
#include <string>

namespace ros2_ign_bridge
{

struct BridgeRos2toIgnHandles
{
  rclcpp::SubscriptionBase::SharedPtr ros2_subscriber;
  ignition::transport::Node::Publisher ign_publisher;
};

struct BridgeIgntoRos2Handles
{
  std::shared_ptr<ignition::transport::Node> ign_subscriber;
  rclcpp::PublisherBase::SharedPtr ros2_publisher;
};

struct BridgeHandles
{
  BridgeRos2toIgnHandles bridgeRos2toIgn;
  BridgeIgntoRos2Handles bridgeIgntoRos2;
};

BridgeRos2toIgnHandles
create_bridge_from_ros2_to_ign(
  rclcpp::Node::SharedPtr ros2_node,
  std::shared_ptr<ignition::transport::Node> ign_node,
  const std::string & ros2_type_name,
  const std::string & ros2_topic_name,
  const std::string & ign_type_name,
  const std::string & ign_topic_name,
  size_t queue_size)
{
  auto factory = get_factory(ros2_type_name, ign_type_name);
  auto ign_pub = factory->create_ign_publisher(ign_node, ign_topic_name);

  auto ros2_sub = factory->create_ros2_subscriber(ros2_node, ros2_topic_name, queue_size, ign_pub);

  BridgeRos2toIgnHandles handles;
  handles.ros2_subscriber = ros2_sub;
  handles.ign_publisher = ign_pub;
  return handles;
}

BridgeIgntoRos2Handles
create_bridge_from_ign_to_ros2(
  std::shared_ptr<ignition::transport::Node> ign_node,
  rclcpp::Node::SharedPtr ros2_node,
  const std::string & ign_type_name,
  const std::string & ign_topic_name,
  const std::string & ros2_type_name,
  const std::string & ros2_topic_name,
  size_t queue_size)
{
  auto factory = get_factory(ros2_type_name, ign_type_name);
  auto ros2_pub = factory->create_ros2_publisher(ros2_node, ros2_topic_name, queue_size);

  factory->create_ign_subscriber(ign_node, ign_topic_name, ros2_pub);

  BridgeIgntoRos2Handles handles;
  handles.ign_subscriber = ign_node;
  handles.ros2_publisher = ros2_pub;
  return handles;
}

BridgeHandles
create_bidirectional_bridge(
  rclcpp::Node::SharedPtr ros2_node,
  std::shared_ptr<ignition::transport::Node> ign_node,
  const std::string & ros2_type_name,
  const std::string & ign_type_name,
  const std::string & topic_name,
  size_t queue_size = 10)
{
  BridgeHandles handles;
  handles.bridgeRos2toIgn = create_bridge_from_ros2_to_ign(
    ros2_node, ign_node,
    ros2_type_name, topic_name, ign_type_name, topic_name, queue_size);
  handles.bridgeIgntoRos2 = create_bridge_from_ign_to_ros2(
    ign_node, ros2_node,
    ign_type_name, topic_name, ros2_type_name, topic_name, queue_size);
  return handles;
}

}  // namespace ros2_ign_bridge

#endif  // ROS2_IGN_BRIDGE__BRIDGE_HPP_
