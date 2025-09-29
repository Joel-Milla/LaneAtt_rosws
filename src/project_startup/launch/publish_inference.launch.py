from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

import os

def generate_launch_description():
    conda_env = os.path.expanduser('~/miniconda3/envs/ros_ws')
    
    # Get current PYTHONPATH if it exists
    current_pythonpath = os.environ.get('PYTHONPATH', '')
    
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
        
        # Control node
        # Node(
        #     package='control_robot',
        #     executable='control_basic',
        #     name='control_basic_node',
        #     output='screen',
        #     emulate_tty=True,
        # ),
    ])