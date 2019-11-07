#include <chrono>
#include <cstdio>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "std_msgs/msg/string.hpp"

#include "hello_world_cpp/visibility_control.h"

using namespace std::chrono_literals;

namespace hello_world_cpp{
  class Talker : public rclcpp::Node{
    public:
      HELLO_WORLD_CPP_PUBLIC

      explicit Talker(const rclcpp::NodeOptions &options)
        : Node("talker", options)
      {
        setvbuf(stdout, NULL, _IONBF, BUFSIZ);

        auto publish_message = 
          [this]() -> void{
            msg_ = std::make_unique<std_msgs::msg::String>();
            msg_->data = "Hello world: " + std::to_string(count_++);

            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg_->data.c_str());

            pub_->publish(std::move(msg_));
          };

        pub_ = this->create_publisher<std_msgs::msg::String>("chatter", rclcpp::SystemDefaultsQoS());
        timer_ = this->create_wall_timer(1s, publish_message);
      }

    private:
      size_t count_ = 1;
      std::unique_ptr<std_msgs::msg::String> msg_;
      rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
      rclcpp::TimerBase::SharedPtr timer_;
  }; // class Talker
} // namespace hello_world_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(hello_world_cpp::Talker)
