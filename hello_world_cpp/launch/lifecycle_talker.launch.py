import launch
import launch_ros


def generate_launch_description():
    descriptions = launch.LaunchDescription()

    lifecycle_talker_container = launch_ros.actions.ComposableNodeContainer(
            node_name='lifecycle_talker_container',
            node_namespace='',
            package='rclcpp_components',
            node_executable='component_container',
            composable_node_descriptions=[
                launch_ros.descriptions.ComposableNode(
                    package='hello_world_cpp',
                    node_plugin='hello_world_cpp::LifecycleTalker',
                    node_name='lifecycle_talker'
                    )
                ],
            output='screen'
            )

    descriptions.add_action(lifecycle_talker_container)

    return descriptions
