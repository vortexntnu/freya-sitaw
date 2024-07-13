import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    default_params_file = os.path.join(get_package_share_directory('freya_sitaw_setup'),'config','target_tracking_params.yaml')
    params_file = LaunchConfiguration('params_file')
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='name or path to the parameters file to use.'
    )
    
    target_tracking_node = Node(
        package='target_tracking',
        executable='target_tracking_node',
        name='target_tracking_node',
        parameters=[params_file],
        output='screen',
    )
    
    enable_pcl_detector = LaunchConfiguration('pcl_detector')
    enable_pcl_detector_arg = DeclareLaunchArgument(
        'pcl_detector',
        default_value='true',
        description='enable PCL detector'
    )

    pcl_detector_node = Node(
            package='pcl_detector',
            executable='pcl_detector_node',
            name='pcl_detector_node',
            parameters=[os.path.join(get_package_share_directory('freya_sitaw_setup'),'config','pcl_detector_params.yaml')],
            output='screen',
            condition=IfCondition(enable_pcl_detector)
        )
    
    enable_visualization = LaunchConfiguration('visualization')
    enable_visualization_arg = DeclareLaunchArgument(
        'visualization',
        default_value='false',
        description='enable visualization'
    )
    

    target_tracking_visualization_node = Node(
        package='target_tracking_visualization',
        executable='target_tracking_visualization_node',
        name='target_tracking_visualization_node',
        output='screen',
        condition=IfCondition(enable_visualization),
    )
        
    return LaunchDescription([
        params_file_arg,
        enable_pcl_detector_arg,
        pcl_detector_node,
        enable_visualization_arg,
        target_tracking_node,
        target_tracking_visualization_node,
    ])