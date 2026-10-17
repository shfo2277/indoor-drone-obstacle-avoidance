from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cannyedge_test1',
            executable='canny_edge_detector',
            name='canny_edge_detector',
            output='screen',
            remappings=[
                ('/image_raw', '/camera/color/image_raw')
            ]
        )
    ])
