#include <chrono>
#include <cstdio>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "std_msgs/msg/string.hpp"

#include "hello_world_cpp/visibility_control.h"

using namespace std::chrono_literals;

namespace hello_world_cpp{
  class Parameter : public rclcpp::Node{
    public:
      HELLO_WORLD_CPP_PUBLIC
      explicit Parameter(const rclcpp::NodeOptions &options)
        :Node("talker_with_param", options), decoration_("")
      {
        setvbuf(stdout, NULL, _IONBF, BUFSIZ);

        // Create timer callback function
        auto publish_message = [this]() -> void {
          msg_.data = this->decoration_ + "Hello world!" + this->decoration_;
          RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg_.data.c_str());
          pub_->publish(this->msg_);
        };

        // Create publisher
        pub_ = create_publisher<std_msgs::msg::String>("chatter", rclcpp::SystemDefaultsQoS());
        // Create timer
        timer_ = create_wall_timer(1s, publish_message);

        declare_parameter("decoration");
        auto parameter_callback = [this](const std::vector<rclcpp::Parameter> params)
                                  -> rcl_interfaces::msg::SetParametersResult {
          auto result = rcl_interfaces::msg::SetParametersResult();
          result.successful = false;
          for (auto param : params) {
            if (param.get_name() == "decoration") {
              this->decoration_ = param.as_string();
              result.successful = true;
            }
          }
          return result;
        };

        set_on_parameters_set_callback(parameter_callback);
      }

    private:
      std_msgs::msg::String msg_;
      rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
      rclcpp::TimerBase::SharedPtr timer_;

      std::string decoration_;
  };
}

RCLCPP_COMPONENTS_REGISTER_NODE(hello_world_cpp::Parameter)