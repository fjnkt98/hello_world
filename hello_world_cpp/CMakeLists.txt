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
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hello_world_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

# create shared library for talker node
add_library(talker_component SHARED
  src/talker_component.cpp
)
target_compile_definitions(talker_component
  PRIVATE "HELLO_WORLD_CPP_BUILDING_DLL"
)
ament_target_dependencies(talker_component
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
)
rclcpp_components_register_nodes(talker_component
  "hello_world_cpp::Talker"
)

# create shared library for listener node
add_library(listener_component SHARED
  src/listener_component.cpp)
target_compile_definitions(listener_component
  PRIVATE "HELLO_WORLD_CPP_BUILDING_DLL")
ament_target_dependencies(listener_component
  "rclcpp"
  "rclcpp_components"
  "std_msgs")
rclcpp_components_register_nodes(listener_component
  "hello_world_cpp::Listener")

# create shared library for server node
add_library(server_component SHARED
  src/server_component.cpp)
target_compile_definitions(server_component
  PRIVATE "HELLO_WORLD_CPP_BUILDING_DLL")
ament_target_dependencies(server_component
  "rclcpp"
  "rclcpp_components"
  "hello_world_msgs")
rclcpp_components_register_nodes(server_component
  "hello_world_cpp::Server")

# create shared library for client node
add_library(client_component_async SHARED
  src/client_component_async.cpp)
target_compile_definitions(client_component_async
  PRIVATE "HELLO_WORLD_CPP_BUILDING_DLL")
ament_target_dependencies(client_component_async
  "rclcpp"
  "rclcpp_components"
  "hello_world_msgs")
rclcpp_components_register_nodes(client_component_async
  "hello_world_cpp::ClientAsync")

# create shared library for talker with param node
add_library(parameter_component SHARED
  src/parameter_component.cpp)
target_compile_definitions(parameter_component
  PRIVATE "HELLO_WORLD_CPP_BUILDING_DLL")
ament_target_dependencies(parameter_component
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
)
rclcpp_components_register_nodes(parameter_component
  "hello_world_cpp::Parameter")

# create shared library for lifecycle talker node
add_library(lifecycle_talker_component SHARED
  src/lifecycle_talker_component.cpp)
target_compile_definitions(lifecycle_talker_component
  PRIVATE "HELLO_WORLD_CPP_BUILDING_DLL")
ament_target_dependencies(lifecycle_talker_component
  "rclcpp"
  "rclcpp_components"
  "rclcpp_lifecycle"
  "std_msgs"
)
rclcpp_components_register_nodes(lifecycle_talker_component
  "hello_world_cpp::LifecycleTalker")

install(TARGETS
  talker_component
  listener_component
  server_component
  client_component_async
  parameter_component
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
