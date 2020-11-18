#include <cinttypes>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "hello_world_msgs/srv/add_two_ints.hpp"

#include "hello_world_cpp/visibility_control.h"

namespace hello_world_cpp
{

class ServerNode : public rclcpp::Node
{
public:
  HELLO_WORLD_CPP_PUBLIC
  explicit ServerNode(const rclcpp::NodeOptions & options)
  : Node("add_two_ints_server", options)
  {
    using namespace std::placeholders;
    
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    /*auto handle_add_two_ints =
      [this](const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<hello_world_msgs::srv::AddTwoInts::Request> request,
        std::shared_ptr<hello_world_msgs::srv::AddTwoInts::Response> response) -> void
      {
        (void)request_header;
        RCLCPP_INFO(this->get_logger(), "Incoming request\na: %" PRId64 " b: %" PRId64,
          request->a, request->b);
        response->sum = request->a + request->b;
      };*/
    // Create a service that will use the callback function to handle requests.
    srv_ = create_service<hello_world_msgs::srv::AddTwoInts>("add_two_ints", std::bind(&ServerNode::handle_add_two_ints, this, _1, _2, _3));
  }

private:
  rclcpp::Service<hello_world_msgs::srv::AddTwoInts>::SharedPtr srv_;
  void handle_add_two_ints(const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<hello_world_msgs::srv::AddTwoInts::Request> request,
        std::shared_ptr<hello_world_msgs::srv::AddTwoInts::Response> response)
  {
    (void)request_header;
    RCLCPP_INFO(this->get_logger(), "Incoming request\na: %" PRId64 " b: %" PRId64, request->a, request->b);
    response->sum = request->a + request->b;
  }
};

}  // namespace hello_world_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(hello_world_cpp::ServerNode)