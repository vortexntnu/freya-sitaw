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
    params_file = os.path.join(get_package_share_directory('sensor_fusion_launch'),'config','sensors_config.yaml')

    
    with open(params_file, 'r') as file:
        launch_args = yaml.safe_load(file)
        
    enable_lidar = LaunchConfiguration('enable_lidar')
    enable_lidar_arg = DeclareLaunchArgument(
        'enable_lidar',
        default_value=str(launch_args['lidar']['enable']),
        description='enable LiDAR'
    )
    
    sensor_hostname = LaunchConfiguration('sensor_hostname')
    sensor_hostname_arg = DeclareLaunchArgument(
        'sensor_hostname',
        default_value=launch_args['lidar']['sensor_hostname'],
        description='hostname of the sensor'
    )
    
    lidar_mode = LaunchConfiguration('lidar_mode')
    lidar_mode_arg = DeclareLaunchArgument(
        'lidar_mode',
        default_value=launch_args['lidar']['lidar_mode'],
        description='LiDAR mode'
        choices=['512x10', '512x20', '1024x10', '1024x20', '2048x10', '4096x5']
    )
    
    enable_camera = LaunchConfiguration('enable_camera')
    enable_camera_arg = DeclareLaunchArgument(
        'enable_camera',
        default_value=str(launch_args["camera"]["enable"]),
        description='enable Camera'
    )
    
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ouster_ros'), 'launch', 'sensor.launch.xml')
        ),
        launch_arguments={
            'viz': 'False',
            'sensor_hostname': sensor_hostname,
            'lidar_mode': lidar_mode,
        }.items(),
        condition=IfCondition(enable_lidar)
    )
    
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('zed_wrapper'), 'launch', 'zed2i.launch.py')
        ),
        launch_arguments={
            'camera_model': launch_args["camera"]["camera_model"],
            'config_path': os.path.join(get_package_share_directory('sensor_fusion_launch'), 'config', 'zed_common.yaml')
            
        }
        condition=IfCondition(enable_camera)
    )
    
    
        
    return LaunchDescription([
        enable_lidar_arg,
        sensor_hostname_arg,
        lidar_mode_arg,
        enable_camera_arg,
        lidar_launch,
        camera_launch
    ])
    
generate_launch_description()