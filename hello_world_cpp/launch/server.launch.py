import launch
import launch_ros


def generate_launch_description():
    server_node_container = launch_ros.actions.ComposableNodeContainer(
            node_name='server_node_container',
            node_namespace='',
            package='rclcpp_components',
            node_executable='component_container',
            composable_node_descriptions=[
                launch_ros.descriptions.ComposableNode(
                    package='hello_world_cpp',
                    node_plugin='hello_world_cpp::Server',
                    node_name='server'
                    )
                ],
            output='screen'
            )
    return launch.LaunchDescription([server_node_container])
