import os 
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    launch_rqt_arg = DeclareLaunchArgument('debug', default_value='false', description='Launch RQT GUI')
    launch_real_arg = DeclareLaunchArgument('realsense', default_value='false', description='Using Realsense camera')
    launch_rqt = LaunchConfiguration('debug')
    launch_real = LaunchConfiguration('realsense')

    # Get the packages share directory
    package_share_directory = get_package_share_directory('metrics')
    realsense_share_directory = get_package_share_directory('realsense2_camera')
    realsense_launch_name = 'rs_launch.py'

    #Node Variables
    camera_params_filename = 'calibration.yaml'

    # Get the path to important files
    camera_params_path = os.path.join(package_share_directory, 'config', camera_params_filename)

    # Cam topic name
    camera_topic_name = PythonExpression([
    "'/camera/camera/color/image_raw' if '", LaunchConfiguration('realsense'), "' == 'true' else '/image_raw'"])
    
    camera_frame_name = PythonExpression([
    "'camera_color_optical_frame' if '", LaunchConfiguration('realsense'), "' == 'true' else 'camera_link'"])

    camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        parameters=[{
            'video_device': '/dev/video0',
            'image_width': 1280,
            'image_height': 720,
            'pixel_format': 'mjpeg2rgb',
            'frame_id': 'camera_link',
            'framerate': 30.0,
            'camera_info_url': ParameterValue(f'file://{camera_params_path}', value_type=str)
        }],
        condition=UnlessCondition(launch_real)
    )

    camera_launch_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_share_directory, 'launch', realsense_launch_name)
        ),
        condition=IfCondition(launch_real)
    )

    aruco_opencv  = Node( 
        package='aruco_opencv',  
        executable='aruco_tracker_autostart',  
        name='aruco_tracker',  
        parameters=[  
            {'cam_base_topic': camera_topic_name},  
            {'marker_size': 0.1},  
            # Select the ArUco dictionary (e.g., ARUCO_ORIGINAL, 4X4_50, 4x4_100)  
            {'marker_dict': '4X4_50'},
            {'camera_frame': camera_frame_name} 
        ], 
        output='screen',  
    )  

    aruco_distance = Node(
        package='metrics',
        executable='aruco',
        condition=UnlessCondition(launch_real)
    )

    aruco_distance_realsense = Node(
        package='metrics',
        executable='aruco_realsense',
        condition=IfCondition(launch_real)
    )

    # To change the dps based on the number of data you want per second.
    error_metric = Node(
        package='metrics',
        executable='error',
        parameters=[{
            'dps': 10.0,
        }],
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'camera_link']
    )

    # Rqt nodes 
    camera_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        condition=IfCondition(launch_rqt)
    )
    
    graph_node = Node(
        package='rqt_graph',
        executable='rqt_graph',
        condition=IfCondition(launch_rqt)
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2'
    )

    ld = LaunchDescription([launch_rqt_arg,
                            launch_real_arg,
                            camera_node,
                            camera_launch_real,
                            aruco_opencv,
                            aruco_distance,
                            aruco_distance_realsense,
                            error_metric,
                            static_tf,
                            camera_view,
                            graph_node,
                            rviz
                            ])
    return ld