from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

import os

def generate_launch_description():
    conda_env = os.path.expanduser('~/miniconda3/envs/ros_ws')
    
    return LaunchDescription([
        # Video publisher
        Node(
            package='publish_video',
            executable='publish_video_node',
            name='video_publisher',
            output='screen',
            emulate_tty=True,
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

        # Comment to use cpp
        # Control node
        # Node(
        #     package='control_robot',
        #     executable='control_robot_node',
        #     name='control_robot_node',
        #     output='screen',
        #     emulate_tty=True,
        #     arguments=['--ros-args', '--log-level', 'control_robot_node:=DEBUG'],
        # ),
    ])
    '''
    ros2 launch realsense2_camera rs_launch.py   rgb_camera.color_profile:='1280,720,15'
    '''