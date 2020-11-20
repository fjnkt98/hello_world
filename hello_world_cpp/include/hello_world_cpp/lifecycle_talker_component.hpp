#ifndef HELLO_WORLD_CPP_LIFECYCLE_TALKER_COMPONENT_HPP_
#define HELLO_WORLD_CPP_LIFECYCLE_TALKER_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <std_msgs/msg/string.hpp>

#include "hello_world_cpp/visibility.h"

namespace hello_world_cpp {
  class LifecycleTalker : public rclcpp_lifecycle::LifecycleNode {
    public:
      HELLO_WORLD_CPP_PUBLIC
      explicit LifecycleTalker(const rclcpp::NodeOptions &options);

    private:
      std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> pub_;
      std::shared_ptr<rclcpp::TimerBase> timer_;
      std_msgs::msg::String msg_;
      size_t count_;

      void publishMessage();

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
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
      on_error(const rclcpp_lifecycle::State &);
  };
} // namespace hello_world_cpp

#endif
