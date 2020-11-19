#include <chrono>
#include <cinttypes>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "hello_world_msgs/srv/add_two_ints.hpp"

#include "hello_world_cpp/visibility_control.h"

using namespace std::chrono_literals;

namespace hello_world_cpp
{
class ClientNode : public rclcpp::Node
{
public:
  HELLO_WORLD_CPP_PUBLIC
  explicit ClientNode(const rclcpp::NodeOptions & options)
  : Node("add_two_ints_client", options)
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    client_ = create_client<hello_world_msgs::srv::AddTwoInts>("add_two_ints");

    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }
    auto request = std::make_shared<hello_world_msgs::srv::AddTwoInts::Request>();
    request->a = 2;
    request->b = 3;

    auto future_result = client_->async_send_request(request, std::bind(&ClientNode::response_received_callback, this, std::placeholders::_1));
  }

private:
  rclcpp::Client<hello_world_msgs::srv::AddTwoInts>::SharedPtr client_;
  void response_received_callback(rclcpp::Client<hello_world_msgs::srv::AddTwoInts>::SharedFuture future){
    auto result = future.get();
    RCLCPP_INFO(get_logger(), "Result of add_two_ints: %" PRId64, result->sum);
    rclcpp::shutdown();
  }
};

}  // namespace hello_world_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(hello_world_cpp::ClientNode)