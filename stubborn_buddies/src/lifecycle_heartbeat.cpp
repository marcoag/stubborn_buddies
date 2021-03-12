// Copyright (c) 2020 Mapless AI, Inc.
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

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "stubborn_buddies_msgs/msg/heartbeat.hpp"
#include "stubborn_buddies_msgs/msg/status.hpp"

using namespace std::chrono_literals;

constexpr std::chrono::milliseconds LEASE_DELTA = 20ms; ///< Buffer added to heartbeat to define lease.

constexpr char DEFAULT_HEARTBEAT_NAME[] = "heartbeat";
constexpr char DEFAULT_INACTIVE_HEARTBEAT_NAME[] = "inactive_heartbeat";
constexpr char DEFAULT_ACTIVE_STATUS_NAME[] = "active_status";

namespace lifecycle_heartbeat
{
  
class LifecycleHeartbeat : public rclcpp_lifecycle::LifecycleNode
{
public:
    
  explicit LifecycleHeartbeat(const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode("lifecycle_heartbeat", options),
    heartbeat_topic_(DEFAULT_HEARTBEAT_NAME), active_node_(true),
    heartbeat_period_(200ms), qos_profile_(1),
    active_status_topic_(DEFAULT_ACTIVE_STATUS_NAME)
    
  {
    //Declare parameters
    //by default we are the active node
    this->declare_parameter<bool>("active_node", true);
    //by default 200ms
    this->declare_parameter<int>("heartbeat_period", 200);
    
    configure();
    activate();
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &)
  {  
    //Retreive parameters values
    this->get_parameter("active_node", active_node_);
    heartbeat_period_ = std::chrono::milliseconds(this->get_parameter("heartbeat_period").as_int());  
    
    // Initialize and configure node
    qos_profile_
      .liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC)
      .liveliness_lease_duration(heartbeat_period_ + LEASE_DELTA)
      .deadline(heartbeat_period_ + LEASE_DELTA);

    //publish on /heartbeat if active node otherwise publish on /watchdogs_heartbeat
    //and subscribe to the active node status in case we need to take over
    if(active_node_)
      heartbeat_topic_ = std::string(DEFAULT_HEARTBEAT_NAME);
    else
    {
      heartbeat_topic_ = std::string(DEFAULT_INACTIVE_HEARTBEAT_NAME);
      status_sub_ = this->create_subscription<stubborn_buddies_msgs::msg::Status>(
        active_status_topic_,
        10,
        [this](const typename stubborn_buddies_msgs::msg::Status::SharedPtr msg) -> void {
                RCLCPP_WARN(get_logger(), "Watchdog rised at %s, "
                                          "self reconfiguration triggered",
                            active_status_topic_.c_str(),
                            msg->stamp.sec);
                deactivate();
                cleanup();
                this->set_parameter(rclcpp::Parameter("active_node", true));
                configure();
                activate();
        });
    }

    hb_publisher_ = this->create_publisher<stubborn_buddies_msgs::msg::Heartbeat>(heartbeat_topic_, qos_profile_);

    RCUTILS_LOG_INFO_NAMED(get_name(), "on_configure() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &)
  {
    hb_timer_ = this->create_wall_timer(heartbeat_period_, std::bind(&LifecycleHeartbeat::hb_timer_callback, this));
    
    hb_publisher_->on_activate();
    
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &)
  {
    hb_publisher_->on_deactivate();
    
    if(status_sub_)
    {
      status_sub_.reset(); // there does not seem to be a 'deactivate' for subscribers.
      status_sub_ = nullptr;
    }
    
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
    on_cleanup(const rclcpp_lifecycle::State &)
  {
    
    hb_publisher_.reset();
    hb_timer_.reset();
    
    if(status_sub_)
    {
      status_sub_.reset(); // there does not seem to be a 'deactivate' for subscribers.
      status_sub_ = nullptr;
    }


    RCUTILS_LOG_INFO_NAMED(get_name(), "on_cleanup() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
    on_shutdown(const rclcpp_lifecycle::State &)
  {
    hb_publisher_.reset();
    hb_timer_.reset();
    
    if(status_sub_)
    {
      status_sub_.reset(); // there does not seem to be a 'deactivate' for subscribers.
      status_sub_ = nullptr;
    }

    RCUTILS_LOG_INFO_NAMED(get_name(), "on_shutdown() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  } 

private: 
  
  void hb_timer_callback()
  {
    auto message = stubborn_buddies_msgs::msg::Heartbeat();
    rclcpp::Time now = this->get_clock()->now();
    message.stamp = now;
    RCLCPP_INFO(this->get_logger(), "Publishing heartbeat sent at [%f]",  now.seconds());
    hb_publisher_->publish(message);
  }
  
  rclcpp_lifecycle::LifecyclePublisher<stubborn_buddies_msgs::msg::Heartbeat>::SharedPtr hb_publisher_ = nullptr;
  rclcpp::Subscription<stubborn_buddies_msgs::msg::Status>::SharedPtr status_sub_ = nullptr;
  
  std::string heartbeat_topic_;
  bool active_node_;
  std::chrono::milliseconds heartbeat_period_;
  rclcpp::TimerBase::SharedPtr hb_timer_;
  rclcpp::QoS qos_profile_;
  const std::string active_status_topic_;

};

}// namespace lifecycle_heartbeat

RCLCPP_COMPONENTS_REGISTER_NODE(lifecycle_heartbeat::LifecycleHeartbeat)
