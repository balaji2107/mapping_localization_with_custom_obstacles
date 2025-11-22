import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LifecycleNode

def generate_launch_description():

    pkg_share = get_package_share_directory('ros2_mapping')

    use_sim_time = LaunchConfiguration('use_sim_time')

    
    default_slam_params_file = os.path.join(
        pkg_share, 
        'config', 
        'mapper_params_online_async.yaml'
    )

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')    

    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=default_slam_params_file,
        description='Full path to the SLAM Toolbox parameters file to use',
    )
    
    slam_params_file = LaunchConfiguration('slam_params_file')

    slam_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('slam_toolbox'), 
                'launch', 
                'online_async_launch.py'
            )]),
            launch_arguments={
                # Mapping your declared argument to the one expected by slam_toolbox
                'slam_params_file': slam_params_file, 
                'use_sim_time': use_sim_time
            }.items(),
        )
    
    # slam_launch=LifecycleNode( 
    #     parameters=[
    #     slam_params_file,
    #     {
    #         'use_sim_time': use_sim_time
    #     }
    #     ],
    #     package='slam_toolbox',
    #     executable='async_slam_toolbox_node',
    #     name='slam_toolbox',
    #     output='screen',
    #     namespace='',
    #     autostart=True
    # )
    return LaunchDescription([
            declare_use_sim_time_argument,
            slam_params_file_arg,
            slam_launch
        ])