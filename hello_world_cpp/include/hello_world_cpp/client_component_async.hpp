#ifndef HELLO_WORLD_CPP_CLIENT_COMPONENT_ASYNC_HPP_
#define HELLO_WORLD_CPP_CLIENT_COMPONENT_ASYNC_HPP_

#include <rclcpp/rclcpp.hpp>

#include "hello_world_msgs/srv/add_two_ints.hpp"
#include "hello_world_cpp/visibility.h"

namespace hello_world_cpp {
  class ClientAsync : public rclcpp::Node {
    public:
      HELLO_WORLD_CPP_PUBLIC
      explicit ClientAsync(const rclcpp::NodeOptions &options);

    private:
      // Service client(async)
      rclcpp::Client<hello_world_msgs::srv::AddTwoInts>::SharedPtr client_;

      // Callback function for 'AddTwoInts' service response
      void responseReceivedCallback(rclcpp::Client<hello_world_msgs::srv::AddTwoInts>::SharedFuture future);
  };
} // namespace hello_world_cpp

#endif
