import launch
import launch_ros


def generate_launch_description():
    parameter_node_container = launch_ros.actions.ComposableNodeContainer(
            node_name='parameter_node_container',
            node_namespace='',
            package='rclcpp_components',
            node_executable='component_container',
            composable_node_descriptions=[
                launch_ros.descriptions.ComposableNode(
                    package='hello_world_cpp',
                    node_plugin='hello_world_cpp::Parameter',
                    node_name='talker_with_parameter'
                    )
                ],
            output='screen'
            )
    return launch.LaunchDescription([parameter_node_container])
