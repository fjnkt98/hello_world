#ifndef HELLO_WORLD_CPP__TALKER_HPP_
#define HELLO_WORLD_CPP__TALKER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

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

      void publish_message();
  };
} // namespace hello_world_cpp

#endif // HELLO_WORLD_CPP__TALKER_HPP_
