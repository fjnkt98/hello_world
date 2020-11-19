import launch
import launch_ros


def generate_launch_description():
    client_async_node_container = launch_ros.actions.ComposableNodeContainer(
            node_name='client_async_node_container',
            node_namespace='',
            package='rclcpp_components',
            node_executable='component_container',
            composable_node_descriptions=[
                launch_ros.descriptions.ComposableNode(
                    package='hello_world_cpp',
                    node_plugin='hello_world_cpp::ClientAsync',
                    node_name='client_async'
                    )
                ],
            output='screen'
            )

    return launch.LaunchDescription([client_async_node_container])
