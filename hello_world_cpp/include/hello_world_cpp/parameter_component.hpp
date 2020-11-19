#ifndef HELLO_WORLD_CPP_PARAMETER_COMPONENT_HPP_
#define HELLO_WORLD_CPP_PARAMETER_COMPONENT_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "hello_world_cpp/visibility.h"

namespace hello_world_cpp {
  class Parameter : public rclcpp::Node{
    public:
      HELLO_WORLD_CPP_PUBLIC
      explicit Parameter(const rclcpp::NodeOptions &options);

    private:
      std_msgs::msg::String msg_;
      rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
      rclcpp::TimerBase::SharedPtr timer_;

      std::string decoration_;

      void publishMessage();
      rcl_interfaces::msg::SetParametersResult parameterSetCallback(const std::vector<rclcpp::Parameter> params);
  };
} // namespace hello_world_cpp

#endif
