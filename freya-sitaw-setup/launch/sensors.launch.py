import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():     
    enable_lidar = LaunchConfiguration('enable_lidar')
    enable_lidar_arg = DeclareLaunchArgument(
        'enable_lidar',
        default_value='True',
        description='enable LiDAR',
    )
    
    enable_camera = LaunchConfiguration('enable_camera')
    enable_camera_arg = DeclareLaunchArgument(
        'enable_camera',
        default_value='True',
        description='enable Camera',
    )

    enable_seapath = LaunchConfiguration('enable_seapath')
    enable_seapath_arg = DeclareLaunchArgument(
        'enable_seapath',
        default_value='True',
        description='enable Seapath',
    )
    
    lidar_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ouster_ros'), 'launch', 'driver.launch.py')
        ),
        launch_arguments={
            'params_file': os.path.join(get_package_share_directory('freya_sitaw_setup'), 'config', 'ouster_driver_params.yaml'),
            'viz': 'False',
        }.items(),
        condition=IfCondition(enable_lidar),
    )
    
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('zed_wrapper'), 'launch', 'zed_camera.launch.py')
        ),
        launch_arguments={
            'camera_model': 'zed2i',
            'camera_name' : 'zed',
            'config_path': os.path.join(get_package_share_directory('freya_sitaw_setup'), 'config', 'zed_driver_params.yaml'),
            
        }.items(),
        condition=IfCondition(enable_camera),
    )

    seapath_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('seapath_ros_driver'), 'launch', 'seapath_ros_driver_launch.py')
        ),
        launch_arguments={
            'params_file': os.path.join(get_package_share_directory('freya_sitaw_setup'), 'config', 'seapath_driver_params.yaml'),
        }.items(),
        condition=IfCondition(enable_seapath),
    )
    
    
        
    return LaunchDescription([
        enable_lidar_arg,
        enable_camera_arg,
        enable_seapath_arg,
        lidar_launch,
        camera_launch,
        seapath_launch,
    ])
    
generate_launch_description()