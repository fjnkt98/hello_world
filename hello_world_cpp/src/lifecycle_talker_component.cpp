#include "hello_world_cpp/lifecycle_talker_component.hpp"

#include <chrono>
#include <cstdio>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <std_msgs/msg/string.hpp>

namespace hello_world_cpp {
  LifecycleTalker::LifecycleTalker(const rclcpp::NodeOptions &options)
    : rclcpp_lifecycle::LifecycleNode("lifecycle_talker", options),
      count_(0)
  {
    // At the lifecycle node, Do nothing in constructor
  }

  /* Callback function for state 'Configuring' */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  LifecycleTalker::on_configure(const rclcpp_lifecycle::State &) {
    using namespace std::chrono_literals;

    // No stdout buffering
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Create lifecycle publisher
    rclcpp::QoS qos(10);
    pub_ = create_publisher<std_msgs::msg::String>("lifecycle_chatter", qos);
    
    // Create timer
    timer_ = create_wall_timer(1s, std::bind(&LifecycleTalker::publishMessage, this));

    RCLCPP_INFO(get_logger(), "on_configure() is called.");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  /* Callback function for state 'Activating' */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  LifecycleTalker::on_activate(const rclcpp_lifecycle::State &) {
    pub_->on_activate();

    RCLCPP_INFO(get_logger(), "on_activate() is called.");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  /* Callback function for state 'Deactivating' */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  LifecycleTalker::on_deactivate(const rclcpp_lifecycle::State &) {
    pub_->on_deactivate();
    
    RCLCPP_INFO(get_logger(), "on_deactivate() is called.");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  /* Callback function for state 'Cleaning Up' */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  LifecycleTalker::on_cleanup(const rclcpp_lifecycle::State &) {
    timer_.reset();
    pub_.reset();

    RCLCPP_INFO(get_logger(), "on_cleanup() is called.");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  /* Callback function for state 'Shutting Down' */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  LifecycleTalker::on_shutdown(const rclcpp_lifecycle::State &state) {
    timer_.reset();
    pub_.reset();

    RCLCPP_INFO(get_logger(), "on_shutdown() is called from state %s.", state.label().c_str());

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  LifecycleTalker::on_error(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_logger(), "on_error() is called.");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  void LifecycleTalker::publishMessage(){
    count_++;
    msg_.data = "Hello Lifecycle!: " + std::to_string(count_);

    if (!pub_->is_activated()) {
      RCLCPP_INFO(get_logger(), "LifecycleTalker is currently inactive. ");
    } else {
      RCLCPP_INFO(get_logger(), "LifecycleTalker is active. '%s'", msg_.data.c_str());
    }

    pub_->publish(msg_);
  }
} // namespace hello_world_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(hello_world_cpp::LifecycleTalker)
