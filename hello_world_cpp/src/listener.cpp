#include <cstdio>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"

#include "hello_world_cpp/visibility_control.h"

namespace hello_world_cpp{
  class Listener : public rclcpp::Node{
    public:
      HELLO_WORLD_CPP_PUBLIC

      explicit Listener(const rclcpp::NodeOptions &options)
        :Node("listener", options)
      {
        setvbuf(stdout, NULL, _IONBF, BUFSIZ);
        sub_ = create_subscription<std_msgs::msg::String>("chatter", rclcpp::SystemDefaultsQoS(), std::bind(&Listener::callback, this, std::placeholders::_1));
      }

    private:
      rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
      void callback(const std_msgs::msg::String::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(), "I heard: [%s]", msg->data.c_str());
      }
  }; // class Listener
} //namespace hello_world_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(hello_world_cpp::Listener)
