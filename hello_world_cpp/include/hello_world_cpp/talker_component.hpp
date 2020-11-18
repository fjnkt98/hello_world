#ifndef HELLO_WORLD_CPP_TALKER_COMPONENT_HPP_
#define HELLO_WORLD_CPP_TALKER_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <hello_world_cpp/visibility.h>

namespace hello_world_cpp{
  class Talker : public rclcpp::Node{
    public:
      HELLO_WORLD_CPP_PUBLIC
      explicit Talker(const rclcpp::NodeOptions &options);

    private:
      size_t count_;
      std_msgs::msg::String msg_;
      rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
      rclcpp::TimerBase::SharedPtr timer_;

      void publishMessage();
  };
} // namespace hello_world_cpp

#endif // HELLO_WORLD_CPP_TALKER_COMPONENT_HPP_
