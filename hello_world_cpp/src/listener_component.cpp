#include <cstdio>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/string.hpp>

#include "hello_world_cpp/listener_component.hpp"

namespace hello_world_cpp{
  Listener::Listener(const rclcpp::NodeOptions &options)
    :Node("listener", options)
  {
    // No stdout buffering
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Create subscription with QoS whose depth_history is 10
    rclcpp::QoS qos(10);
    sub_ = create_subscription<std_msgs::msg::String>("chatter", qos, std::bind(&Listener::callback, this, std::placeholders::_1));
  }

  void Listener::callback(const std_msgs::msg::String::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "I heard: [%s]", msg->data.c_str());
  }
} //namespace hello_world_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(hello_world_cpp::Listener)
