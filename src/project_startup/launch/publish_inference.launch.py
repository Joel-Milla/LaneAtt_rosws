from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

import os

def generate_launch_description():
    conda_env = os.path.expanduser('~/miniconda3/envs/ros_ws')
    
    # Get current PYTHONPATH if it exists
    current_pythonpath = os.environ.get('PYTHONPATH', '')
    new_pythonpath = f"{conda_env}/lib/python3.10/site-packages:{current_pythonpath}"
    
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
        ExecuteProcess(
            cmd=[
                'bash', '-c',
                f'export PYTHONPATH={conda_env}/lib/python3.10/site-packages:$PYTHONPATH && '
                'ros2 run inference make_prediction_node'
            ],
            output='screen',
        ),
    ])