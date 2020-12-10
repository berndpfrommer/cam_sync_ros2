#
# example launch file for cam sync
#

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """launch cam_sync."""
    container = ComposableNodeContainer(
            name='cam_sync_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='cam_sync_ros2',
                    plugin='cam_sync_ros2::CamSync',
                    name='sync',
                    parameters=[],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
            ],
            output='screen',
    )
    return launch.LaunchDescription([container])
