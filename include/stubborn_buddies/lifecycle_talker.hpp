// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#ifndef FAILOVER__TALKER_COMPONENT_HPP_
#define FAILOVER__TALKER_COMPONENT_HPP_

#include <chrono>

#include "stubborn_buddies/visibility_control.h"
#include "stubborn_buddies/stubborn_values.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "std_msgs/msg/string.hpp"

#include "sw_watchdog_msgs/msg/status.hpp"

using namespace std::chrono_literals;

constexpr char DEFAULT_ACTIVE_STATUS_NAME[] = "active_status";

namespace lifecycle_talker
{

class LifecycleTalker : public rclcpp_lifecycle::LifecycleNode
{
public:
  COMPOSITION_PUBLIC
  explicit LifecycleTalker(const rclcpp::NodeOptions& options);
  
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
    on_cleanup(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
    on_shutdown(const rclcpp_lifecycle::State &state);

private:
  
  void talker_timer_callback();
  
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr talker_publisher_ = nullptr;
  rclcpp::Subscription<sw_watchdog_msgs::msg::Status>::SharedPtr status_sub_ = nullptr;
  
  bool active_node_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
  std::chrono::milliseconds talker_period_;
  const std::string active_status_topic_;
  
};

}  // namespace composition

#endif  // COMPOSITION__TALKER_COMPONENT_HPP_
