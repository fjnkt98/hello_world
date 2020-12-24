cmake_minimum_required(VERSION 3.5)
project(hello_world_cpp)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# create shared library for talker node
ament_auto_add_library(talker_component SHARED
  src/talker_component.cpp)
rclcpp_components_register_nodes(talker_component
  "hello_world_cpp::Talker")

# create shared library for listener node
ament_auto_add_library(listener_component SHARED
  src/listener_component.cpp)
rclcpp_components_register_nodes(listener_component
  "hello_world_cpp::Listener")

# create shared library for server node
ament_auto_add_library(server_component SHARED
  src/server_component.cpp)
rclcpp_components_register_nodes(server_component
  "hello_world_cpp::Server")

# create shared library for client node
ament_auto_add_library(client_component_async SHARED
  src/client_component_async.cpp)
rclcpp_components_register_nodes(client_component_async
  "hello_world_cpp::ClientAsync")

# create shared library for lifecycle talker node
ament_auto_add_library(lifecycle_talker_component SHARED
  src/lifecycle_talker_component.cpp)
rclcpp_components_register_nodes(lifecycle_talker_component
  "hello_world_cpp::LifecycleTalker")

install(TARGETS
  talker_component
  listener_component
  server_component
  client_component_async
  lifecycle_talker_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
