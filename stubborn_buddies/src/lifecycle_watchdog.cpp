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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "stubborn_buddies_msgs/msg/heartbeat.hpp"
#include "stubborn_buddies_msgs/msg/status.hpp"

constexpr char DEFAULT_HEARTBEAT_NAME[] = "heartbeat";
constexpr char DEFAULT_INACTIVE_HEARTBEAT_NAME[] = "inactive_heartbeat";
constexpr char DEFAULT_INACTIVE_STATUS_NAME[] = "inactive_status";
constexpr char DEFAULT_ACTIVE_STATUS_NAME[] = "active_status";

using namespace std::chrono_literals;

namespace lifecycle_watchdog
{
  
class LifecycleWatchdog : public rclcpp_lifecycle::LifecycleNode
{
public:

  explicit LifecycleWatchdog(const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode("lifecycle_watchdog", options),
    active_node_(true), qos_profile_(1), status_topic_(),
    heartbeat_topic_(DEFAULT_INACTIVE_HEARTBEAT_NAME), lease_duration_(400ms)
  {
    //Declare parameters
    //by default we are the active node
    this->declare_parameter<bool>("active_node", true);
    //by default 200ms
    this->declare_parameter<int>("lease_duration", 400);
    //command to run the innactive node
    this->declare_parameter<std::string>("innactive_node_command", "ros2 run stubborn_buddies linktime_composition --ros-args -p active_node:=false&");
    
    configure();
    activate();
  }

  /// Publish lease expiry of the watched entity
  void missed_hearbeat()
  {
    
    auto msg = std::make_unique<stubborn_buddies_msgs::msg::Status>();
    rclcpp::Time now = this->get_clock()->now();
    msg->stamp = now;
    msg->missed_number = 1;

    // Print the current state for demo purposes
    if (!status_pub_->is_activated()) {
        RCLCPP_INFO(get_logger(),
                    "Lifecycle publisher is currently inactive. Messages are not published.");
    } else {
        RCLCPP_INFO(get_logger(),
                    "Publishing lease expiry (missed count: %u) at [%f]",
                    msg->missed_number, now.seconds());
        status_pub_->publish(std::move(msg));
    }
    
    if(active_node_)
    {
      RCLCPP_WARN(get_logger(), "Got a missed hb at %s, spawning a " 
                                "new inactive process",
                  status_topic_.c_str());
      system(innactive_node_command_.c_str());
    }
  }


  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &)
  {
    //Retrieve parameters
    this->get_parameter("active_node", active_node_);
    lease_duration_ = std::chrono::milliseconds(this->get_parameter("lease_duration").as_int()); 
    this->get_parameter("innactive_node_command", innactive_node_command_);
    
    qos_profile_
        .liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC)
        .liveliness_lease_duration(lease_duration_);
        
    heartbeat_sub_options_.event_callbacks.liveliness_callback = 
            [this](rclcpp::QOSLivelinessChangedInfo &event) -> void {
              printf("Reader Liveliness changed event: \n");
              printf("  alive_count: %d\n", event.alive_count);
              printf("  not_alive_count: %d\n", event.not_alive_count);
              printf("  alive_count_change: %d\n", event.alive_count_change);
              printf("  not_alive_count_change: %d\n", event.not_alive_count_change);
              if(event.alive_count == 0) {
                  missed_hearbeat();
              }
          };
      
      //If we are the active node we watch for the inactive heartbeat
      if(active_node_)
      {
        heartbeat_topic_ = std::string(DEFAULT_INACTIVE_HEARTBEAT_NAME);
        status_topic_ = std::string(DEFAULT_INACTIVE_STATUS_NAME);
        //spawn the inactive node
        system(innactive_node_command_.c_str());
      }
      else
      {
        heartbeat_topic_ = std::string(DEFAULT_HEARTBEAT_NAME);
        status_topic_ = std::string(DEFAULT_ACTIVE_STATUS_NAME);
        //we need to suscribe to our own topic to be able to reconfigure
        //without breaking the other callbacks, which is what happens if
        //we call cleanup() in missed_heartbeat()
        status_sub_ = this->create_subscription<stubborn_buddies_msgs::msg::Status>(
        status_topic_,
        10,
        [this](const typename stubborn_buddies_msgs::msg::Status::SharedPtr msg) -> void {
                RCLCPP_WARN(get_logger(), "Watchdog rised at %s, "
                                          "self reconfiguration triggered",
                            status_topic_.c_str(),
                            msg->stamp.sec);
                deactivate();
                cleanup();
                this->set_parameter(rclcpp::Parameter("active_node", true));
                configure();
                activate();
        });
      }

    RCUTILS_LOG_INFO_NAMED(get_name(), "on_configure() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &)
  {
    if(!heartbeat_sub_) {
      heartbeat_sub_ = create_subscription<stubborn_buddies_msgs::msg::Heartbeat>(
        heartbeat_topic_,
        qos_profile_,
        [this](const typename stubborn_buddies_msgs::msg::Heartbeat::SharedPtr msg) -> void {
            RCLCPP_INFO(get_logger(), "Watching %s, heartbeat sent at [%d.x]", heartbeat_topic_.c_str(), msg->stamp.sec);
        },
        heartbeat_sub_options_);
    }
    
    status_pub_ = create_publisher<stubborn_buddies_msgs::msg::Status>(status_topic_, 10);
    status_pub_->on_activate();
    
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &)
  {
    heartbeat_sub_.reset(); // there does not seem to be a 'deactivate' for subscribers.
    heartbeat_sub_ = nullptr;
    
    if(status_sub_)
    {
      status_sub_.reset(); // there does not seem to be a 'deactivate' for subscribers.
      status_sub_ = nullptr;
    }
    
    status_pub_->on_deactivate();
    
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
    on_cleanup(const rclcpp_lifecycle::State &)
  {
    heartbeat_sub_.reset(); // there does not seem to be a 'deactivate' for subscribers.
    heartbeat_sub_ = nullptr;
    
    if(status_sub_)
    {
      status_sub_.reset(); // there does not seem to be a 'deactivate' for subscribers.
      status_sub_ = nullptr;
    }
    
    status_pub_.reset();
    
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_cleanup() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
    on_shutdown(const rclcpp_lifecycle::State &)
  {
    heartbeat_sub_.reset(); // there does not seem to be a 'deactivate' for subscribers.
    heartbeat_sub_ = nullptr;
    
    if(status_sub_)
    {
      status_sub_.reset(); // there does not seem to be a 'deactivate' for subscribers.
      status_sub_ = nullptr;
    }
    
    status_pub_.reset();

    RCUTILS_LOG_INFO_NAMED(get_name(), "on_shutdown() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }
  
private:
  
  rclcpp_lifecycle::LifecyclePublisher<stubborn_buddies_msgs::msg::Status>::SharedPtr status_pub_ = nullptr;
  rclcpp::Subscription<stubborn_buddies_msgs::msg::Heartbeat>::SharedPtr heartbeat_sub_ = nullptr;
  rclcpp::Subscription<stubborn_buddies_msgs::msg::Status>::SharedPtr status_sub_ = nullptr;
    
  bool active_node_;
  rclcpp::QoS qos_profile_;
  rclcpp::SubscriptionOptions heartbeat_sub_options_;
  std::string status_topic_;
  std::string heartbeat_topic_;
  std::string innactive_node_command_;
  /// The lease duration granted to the remote (heartbeat) publisher
  std::chrono::milliseconds lease_duration_;
  
};

} // namespace lifecycle_watchdog

RCLCPP_COMPONENTS_REGISTER_NODE(lifecycle_watchdog::LifecycleWatchdog)
