#ifndef HELLO_WORLD_CPP__LISTENER_HPP_
#define HELLO_WORLD_CPP__LISTENER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.h>

namespace hello_world_cpp{
  class Listener : public rclcpp::Node{
    public:
      HELLO_WORLD_CPP_PUBLIC

      explicit Listener(const rclcpp::NodeOptions &options);

    private:
      rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;

      void callback(const std_msgs::msg::String::SharedPtr msg);
  };
} // namespace hello_world_cpp

#endif // HELLO_WORLD_CPP__LISTENER_HPP_
