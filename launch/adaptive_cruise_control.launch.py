from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer , Node
from launch_ros.descriptions import ComposableNode
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

    republisher_node = Node(
        package='image_transport',
        executable='republish',
        name='image_republisher',
        arguments=['compressed','raw'],
        remappings=[
            ('in/compressed', '/camera/image_raw/compressed'),
            ('out', '/camera/image_raw/decompressed')
        ]
    )

    yolo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('yolo_ros2'),
                'launch',
                'yolo_ros2.launch.py'
            )
        )
    )

    return LaunchDescription([
        container,
        republisher_node,
        yolo_launch
        ])