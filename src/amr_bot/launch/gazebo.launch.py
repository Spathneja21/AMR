"""Robot side of the Gazebo backend: spawns the sensor-stripped robot (Gazebo's own lidar/camera
crash the server here - see PROCESS.md), bridges /cmd_vel, /odom, /tf, /joint_states, /clock to ROS 2
with the same names mujoco_bridge uses, and starts robot_state_publisher, gazebo_lidar (/scan) and
gazebo_camera (/camera/image_raw).
Everything downstream must run with use_sim_time:=true, because these stamps are Gazebo's clock.

Start `ign gazebo -r .../office.world.sdf` yourself first (it's an interactive GUI window, same as
mujoco_bridge's own viewer). Then:
    ros2 launch amr_bot gazebo.launch.py
"""
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    urdf_path = '/ros2_ws/src/amr_bot/urdf/supermarketbot.urdf'
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    # ros_gz_sim create is a one-shot CLI tool, not a long-running node - ExecuteProcess runs it
    # like any other step and moves on once it exits.
    spawn = ExecuteProcess(
        cmd=['bash', '-c',
             'python3 /ros2_ws/src/amr_bot/scripts/urdf_gazebo_nosensors.py > /tmp/supermarketbot_gazebo.urdf '
             '&& ros2 run ros_gz_sim create -world office -file /tmp/supermarketbot_gazebo.urdf '
             '-name supermarketbot -z 0.05'],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/ground_truth/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
        ],
        output='screen',
    )

    lidar = Node(package='amr_bot', executable='gazebo_lidar', output='screen')
    camera = Node(package='amr_bot', executable='gazebo_camera', output='screen')

    return LaunchDescription([rsp, spawn, bridge, lidar, camera])
