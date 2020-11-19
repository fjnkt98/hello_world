#include "hello_world_cpp/client_component_async.hpp"

#include <chrono>
#include <cstdio>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "hello_world_msgs/srv/add_two_ints.hpp"

namespace hello_world_cpp {
  ClientAsync::ClientAsync(const rclcpp::NodeOptions &options)
    : Node("add_two_ints_client", options)
  {
    using namespace std::chrono_literals;

    // No stdout buffering
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Create service client
    client_ = create_client<hello_world_msgs::srv::AddTwoInts>("add_two_ints");

    // Wait for launching servicve server
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }

    // Create service request
    auto request = std::make_shared<hello_world_msgs::srv::AddTwoInts::Request>();
    // Substitute values into request message
    request->a = 2;
    request->b = 3;

    // Send request to server
    // and set callback function
    auto future_result = client_->async_send_request(request, std::bind(&ClientAsync::responseReceivedCallback, this, std::placeholders::_1));
  }

  void ClientAsync::responseReceivedCallback(rclcpp::Client<hello_world_msgs::srv::AddTwoInts>::SharedFuture future){
    // Wait for the response to come back and be assigned to the future object
    auto result = future.get();
    RCLCPP_INFO(get_logger(), "Result of add_two_ints: %ld", static_cast<uint64_t>(result->sum));
    // Node will exit when response comes back
    rclcpp::shutdown();
  }
}  // namespace hello_world_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(hello_world_cpp::ClientAsync)
