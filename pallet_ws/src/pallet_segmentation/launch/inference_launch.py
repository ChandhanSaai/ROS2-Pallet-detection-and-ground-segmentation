from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pallet_segmentation',
            executable='inference',
            name='pallet_detection_node',
            output='screen'
        ),
        Node(
            package='pallet_segmentation',
            executable='inference_segment',
            name='ground_segmentation_node',
            output='screen'
        )
    ])
