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
find_package(hello_world_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

# create shared library for talker node
add_library(talker SHARED
  src/talker.cpp)
target_compile_definitions(talker
  PRIVATE "HELLO_WORLD_CPP_BUILDING_DLL")
ament_target_dependencies(talker
  "rclcpp"
  "rclcpp_components"
  "std_msgs")
rclcpp_components_register_nodes(talker
  "hello_world_cpp::Talker")

# create shared library for listener node
add_library(listener SHARED
  src/listener.cpp)
target_compile_definitions(listener
  PRIVATE "HELLO_WORLD_CPP_BUILDING_DLL")
ament_target_dependencies(listener
  "rclcpp"
  "rclcpp_components"
  "std_msgs")
rclcpp_components_register_nodes(listener
  "hello_world_cpp::Listener")

install(TARGETS
  talker
  listener
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

ament_package()
