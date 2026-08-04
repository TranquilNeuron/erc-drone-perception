from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'launch', 'depthai_ros_driver', 'camera.launch.py',
                 'camera.i_rgb_resolution:=720p', 'camera.i_rgb_fps:=30.0',
                 'stereo.i_subpixel:=true'],
            output='screen'),
        Node(package='erc_perception', executable='probe_node', output='screen'),
    ])
