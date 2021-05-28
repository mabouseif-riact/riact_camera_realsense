from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='riact_camera_realsense',
            namespace='realsense_driver',
            executable='pointcloud_publisher',
            name='realsense_driver'),
    ])
