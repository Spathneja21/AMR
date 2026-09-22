from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    urdf_path = '/ros2_ws/src/amr_bot/urdf/supermarketbot.urdf'
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description':robot_description}]
        )
    ])