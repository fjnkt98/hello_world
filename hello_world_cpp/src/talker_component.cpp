#include <chrono>
#include <cstdio>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp/logging.hpp>
#include <std_msgs/msg/string.hpp>

#include "hello_world_cpp/talker_component.hpp"


namespace hello_world_cpp{
  Talker::Talker(const rclcpp::NodeOptions &options)
    : Node("talker", options), count_(1)
  {
    using namespace std::chrono_literals;

    // No stdout buffering
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Create publisher with QoS whose history_depth is 10
    // Topic name is 'chatter'
    rclcpp::QoS qos(10);
    pub_ = this->create_publisher<std_msgs::msg::String>("chatter", qos);

    // Create timer with frequency 1[Hz]
    // Binding member function as a timer-callback function
    timer_ = this->create_wall_timer(1s, std::bind(&Talker::publishMessage, this));
  }


  void Talker::publishMessage(){
    count_++;
    msg_.data = "Hello world: " + std::to_string(count_);

    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg_.data.c_str());
    pub_->publish(msg_);
  }
} // namespace hello_world_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(hello_world_cpp::Talker)
