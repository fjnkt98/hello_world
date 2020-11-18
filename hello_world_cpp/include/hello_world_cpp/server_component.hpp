#ifndef HELLO_WORLD_CPP_SERVER_COMPONENT_HPP_
#define HELLO_WORLD_CPP_SERVER_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>

#include "hello_world_msgs/srv/add_two_ints.hpp"
#include "hello_world_cpp/visibility.h"

namespace hello_world_cpp {
  class Server : public rclcpp::Node {
    public:
      HELLO_WORLD_CPP_PUBLIC
      explicit Server(const rclcpp::NodeOptions &options);

    private:
      // Service server
      rclcpp::Service<hello_world_msgs::srv::AddTwoInts>::SharedPtr srv_;

      // Callback function for 'AddTwoInts' service
      void addTwoIntsCallback(const std::shared_ptr<rmw_request_id_t> request_header,
          const std::shared_ptr<hello_world_msgs::srv::AddTwoInts::Request> request,
          std::shared_ptr<hello_world_msgs::srv::AddTwoInts::Response> response);
  };
} // namespace hello_world_cpp


#endif
