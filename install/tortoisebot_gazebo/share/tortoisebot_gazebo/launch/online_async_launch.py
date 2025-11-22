import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    slam_static_parameters = {
        'base_frame': 'base_link', 
        'odom_frame': 'odom', 
        'scan_buffer_size': 50, 
        'scan_topic': '/scan',
    }
    
    # 2. Define dynamic parameters (those using substitutions)
    slam_dynamic_parameters = {
        'use_sim_time': use_sim_time, 
    }

    slam_tool_box_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node', 
        name='slam_toolbox',
        parameters=[
            slam_static_parameters,  # Pass the static dictionary
            slam_dynamic_parameters, # Pass the dynamic dictionary
        ],
        output='screen'
    )


    return LaunchDescription([
        use_sim_time_arg,
        slam_tool_box_node
    ])