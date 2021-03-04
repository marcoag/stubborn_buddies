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

#ifndef FAILOVER__LIFECYCLE_WATCHDOG_HPP_
#define FAILOVER__LIFECYCLE_WATCHDOG_HPP_

#include "stubborn_buddies/visibility_control.h"
#include "stubborn_buddies/stubborn_values.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "sw_watchdog_msgs/msg/heartbeat.hpp"
#include "sw_watchdog_msgs/msg/status.hpp"

using namespace std::chrono_literals;

namespace lifecycle_watchdog
{

class LifecycleWatchdog : public rclcpp_lifecycle::LifecycleNode
{
public:
  COMPOSITION_PUBLIC
  explicit LifecycleWatchdog(const rclcpp::NodeOptions& options);

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
    
  void missed_hearbeat();

private:
  
  rclcpp_lifecycle::LifecyclePublisher<sw_watchdog_msgs::msg::Status>::SharedPtr status_pub_ = nullptr;
  rclcpp::Subscription<sw_watchdog_msgs::msg::Heartbeat>::SharedPtr heartbeat_sub_ = nullptr;
  rclcpp::Subscription<sw_watchdog_msgs::msg::Status>::SharedPtr status_sub_ = nullptr;
    
  bool active_node_;
  rclcpp::QoS qos_profile_;
  rclcpp::SubscriptionOptions heartbeat_sub_options_;
  std::string status_topic_;
  std::string heartbeat_topic_;
  /// The lease duration granted to the remote (heartbeat) publisher
  std::chrono::milliseconds lease_duration_;
  
};

}
#endif  // COMPOSITION__LIFECYCLE_WATCHDOG_HPP_
