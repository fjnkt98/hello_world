#include "hello_world_cpp/parameter_component.hpp"

#include <chrono>
#include <cstdio>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/string.hpp>

namespace hello_world_cpp {
  Parameter::Parameter(const rclcpp::NodeOptions &options)
    : Node("parameter", options),
      decoration_("")
  {
    using namespace std::chrono_literals;

    // No stdout buffering
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Create publisher with QoS whose history_depth is 10
    // Topic name is 'chatter'
    rclcpp::QoS qos(10);
    pub_ = create_publisher<std_msgs::msg::String>("chatter", qos);

    // Create timer
    timer_ = create_wall_timer(1s, std::bind(&Parameter::publishMessage, this));

    // Declare ROS2 node parameter 
    // parameter name is 'decoration'
    // default value is blank
    declare_parameter("decoration", "");

    set_on_parameters_set_callback(std::bind(&Parameter::parameterSetCallback, this, std::placeholders::_1));
  }

  void Parameter::publishMessage() {
    msg_.data = decoration_ + "Hello world!" + decoration_;
    RCLCPP_INFO(get_logger(), "Publishing: '%s'", msg_.data.c_str());
    pub_->publish(msg_);
  }

  rcl_interfaces::msg::SetParametersResult Parameter::parameterSetCallback(const std::vector<rclcpp::Parameter> params) {
    // Result for parameter setting
    auto result = rcl_interfaces::msg::SetParametersResult();

    // set default result as false
    result.successful = false;

    // for each parameters;
    for (auto param : params) {
      if (param.get_name() == "decoration") {
        decoration_ = param.as_string();
        result.successful = true;
      }
    }
    return result;
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(hello_world_cpp::Parameter)
