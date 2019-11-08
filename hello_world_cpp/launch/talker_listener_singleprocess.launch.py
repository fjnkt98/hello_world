import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
            node_name = 'my_container',
            node_namespace = '',
            package = 'rclcpp_components',
            node_executable = 'component_container',
            composable_node_descriptions = [
                ComposableNode(
                    package = 'hello_world_cpp',
                    node_plugin = 'hello_world_cpp::Talker',
                    node_name = 'talker'
                    ),
                ComposableNode(
                    package = 'hello_world_cpp',
                    node_plugin = 'hello_world_cpp::Listener',
                    node_name = 'listener'
                    )
                ],
                output = 'screen',
            )
    return launch.LaunchDescription([container])
