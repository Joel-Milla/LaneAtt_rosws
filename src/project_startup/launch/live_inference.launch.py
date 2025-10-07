from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os

def generate_launch_description():
    conda_env = os.path.expanduser('~/miniconda3/envs/ros_ws')
    
    return LaunchDescription([
        # Live stream publisher
        # ros2 launch realsense2_camera rs_launch.py rgb_camera.color_profile:='1280,720,15'
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('realsense2_camera'),
                    'launch',
                    'rs_launch.py'
                ])
            ]),
            launch_arguments={
                'rgb_camera.color_profile': '1280,720,15'
            }.items()
        ),

        # Inference with shell wrapper to preserve environment
        Node(
            package='inference',
            executable='make_prediction_node',
            name='predicted_lanes_publisher',
            output='screen',
            emulate_tty=True,
            # Add conda environment to PYTHONPATH
            additional_env={
                'PYTHONPATH': f"{conda_env}/lib/python3.10/site-packages:{os.environ.get('PYTHONPATH', '')}"
            }
        ),
        Node(
            package='pycontrol',
            executable='control',
            output='screen',
            emulate_tty=True,
        ),
        # Debug node
        Node(
            package='control_robot',
            executable='control_robot_node',
            name='control_robot_node',
            output='screen',
            emulate_tty=True,
        ),
    ])