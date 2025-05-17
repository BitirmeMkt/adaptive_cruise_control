from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_path = get_package_share_directory('adaptive_cruise_control')

    param_file = os.path.join(pkg_path, 'config', 'adaptive_cruise_control.yaml')

    container = ComposableNodeContainer(
        name='adaptive_cruise_control_container',
        namespace='acc',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='adaptive_cruise_control',
                plugin='adaptive_cruise_control::AdaptiveCruiseControl',
                name='adaptive_cruise_control_node',
                parameters=[param_file],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ]
    )

    return LaunchDescription([container])