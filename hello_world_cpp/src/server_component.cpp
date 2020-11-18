#include "hello_world_cpp/server_component.hpp"

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "hello_world_msgs/srv/add_two_ints.hpp"
#include "hello_world_cpp/visibility.h"

namespace hello_world_cpp {
  Server::Server(const rclcpp::NodeOptions &options)
    : Node("add_two_ints_server", options)
  {
    using namespace std::placeholders;
    
    // No stdout buffering
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Create a service that will use the callback function to handle requests.
    srv_ = create_service<hello_world_msgs::srv::AddTwoInts>("add_two_ints", std::bind(&Server::addTwoIntsCallback, this, _1, _2, _3));
  }

  void Server::addTwoIntsCallback(const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<hello_world_msgs::srv::AddTwoInts::Request> request,
        std::shared_ptr<hello_world_msgs::srv::AddTwoInts::Response> response)
  {
    // To avoid linter
    (void)request_header;

    // Calcurate add two ints
    response->sum = request->a + request->b;

    // Debug Statement
    RCLCPP_INFO(get_logger(), "Incoming request a: %ld, b: %ld", request->a, request->b);
    RCLCPP_INFO(get_logger(), "Sending back response: %ld", static_cast<int64_t>(response->sum));
  }
}  // namespace hello_world_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(hello_world_cpp::Server)
