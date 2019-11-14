#include <chrono>
#include <cstdio>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "std_msgs/msg/string.hpp"

#include "hello_world_cpp/visibility_control.h"
#include "hello_world_cpp/talker.hpp"

using namespace std::chrono_literals;

namespace hello_world_cpp{
  Talker::Talker(const rclcpp::NodeOptions &options)
    : Node("talker", options), count_(1)
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    pub_ = this->create_publisher<std_msgs::msg::String>("chatter", rclcpp::SystemDefaultsQoS());
    timer_ = this->create_wall_timer(1s, std::bind(&Talker::publish_message, this));
  }


  void Talker::publish_message(){
    msg_.data = "Hello world: " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg_.data.c_str());
    pub_->publish(msg_);
  }
} // namespace hello_world_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(hello_world_cpp::Talker)
