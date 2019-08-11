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

#ifndef ROS2_IGN_BRIDGE__FACTORY_HPP_
#define ROS2_IGN_BRIDGE__FACTORY_HPP_

#include <ignition/transport/Node.hh>

// include ROS 2
#include <rclcpp/rclcpp.hpp>

#include <ros2_ign_bridge/factory_interface.hpp>

#include <functional>
#include <memory>
#include <string>

namespace ros2_ign_bridge
{

template<typename ROS2_T, typename IGN_T>
class Factory : public FactoryInterface
{
public:
  Factory(
    const std::string & ros2_type_name, const std::string & ign_type_name)
  : ros2_type_name_(ros2_type_name),
    ign_type_name_(ign_type_name)
  {}

  rclcpp::PublisherBase::SharedPtr
  create_ros2_publisher(
    rclcpp::Node::SharedPtr ros2_node,
    const std::string & topic_name)
  {
    std::shared_ptr<rclcpp::Publisher<ROS2_T>> publisher =
      ros2_node->create_publisher<ROS2_T>(topic_name, rclcpp::QoS(rclcpp::KeepLast(1)));
    return publisher;
  }

  ignition::transport::Node::Publisher
  create_ign_publisher(
    std::shared_ptr<ignition::transport::Node> ign_node,
    const std::string & topic_name)
  {
    return ign_node->Advertise<IGN_T>(topic_name);
  }

  rclcpp::SubscriptionBase::SharedPtr
  create_ros2_subscriber(
    rclcpp::Node::SharedPtr ros2_node,
    const std::string & topic_name,
    ignition::transport::Node::Publisher & ign_pub)
  {
    std::function<void(std::shared_ptr<const ROS2_T>)> fn =
      std::bind(&Factory<ROS2_T, IGN_T>::ros2_callback, std::placeholders::_1, ign_pub);
    std::shared_ptr<rclcpp::Subscription<ROS2_T>> subscription =
      ros2_node->create_subscription<ROS2_T>(topic_name, rclcpp::QoS(rclcpp::KeepLast(1)), fn);
    return subscription;
  }

  void
  create_ign_subscriber(
    std::shared_ptr<ignition::transport::Node> node,
    const std::string & topic_name,
    rclcpp::PublisherBase::SharedPtr ros2_pub)
  {
    std::function<void(const IGN_T &,
      const ignition::transport::MessageInfo &)> subCb =
      [this, ros2_pub](const IGN_T & _msg,
        const ignition::transport::MessageInfo & _info)
      {
        // Ignore messages that are published from this bridge.
        if (!_info.IntraProcess()) {
          this->ign_callback(_msg, ros2_pub);
        }
      };

    node->Subscribe(topic_name, subCb);
  }

protected:
  static
  void ros2_callback(
    std::shared_ptr<const ROS2_T> ros2_msg,
    ignition::transport::Node::Publisher & ign_pub)
  {
    IGN_T ign_msg;
    convert_ros2_to_ign(*ros2_msg, ign_msg);
    ign_pub.Publish(ign_msg);
  }

  static
  void ign_callback(
    const IGN_T & ign_msg,
    rclcpp::PublisherBase::SharedPtr ros2_pub)
  {
    ROS2_T ros2_msg;
    convert_ign_to_ros2(ign_msg, ros2_msg);
    std::shared_ptr<rclcpp::Publisher<ROS2_T>> pub =
      std::dynamic_pointer_cast<rclcpp::Publisher<ROS2_T>>(ros2_pub);
    pub->publish(ros2_msg);
  }

public:
  // since convert functions call each other for sub messages they must be
  // public defined outside of the class
  static
  void
  convert_ros2_to_ign(
    const ROS2_T & ros2_msg,
    IGN_T & ign_msg);
  static
  void
  convert_ign_to_ros2(
    const IGN_T & ign_msg,
    ROS2_T & ros2_msg);

  std::string ros2_type_name_;
  std::string ign_type_name_;
};

}  // namespace ros2_ign_bridge

#endif  // ROS2_IGN_BRIDGE__FACTORY_HPP_
