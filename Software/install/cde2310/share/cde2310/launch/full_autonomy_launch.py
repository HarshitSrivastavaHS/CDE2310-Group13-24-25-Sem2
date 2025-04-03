import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the directories of required packages
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    cde2310_dir = get_package_share_directory('cde2310')
    custom_explorer_dir = get_package_share_directory('custom_explorer')  # Corrected package directory

    return LaunchDescription([
        # SLAM Toolbox for mapping/localization
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
            ),
            launch_arguments={'use_sim_time': 'true'}.items(),
        ),

        # Nav2 Navigation Stack (planner, regulated pure pursuit controller, costmaps)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'params_file': os.path.join(cde2310_dir, 'config', 'nav2_params.yaml'),  # Reference custom nav2_params.yaml
            }.items(),
        ),

        # Autonomous Explorer Node (frontier-based exploration)
        Node(
            package='custom_explorer',  # Corrected package name based on screenshot
            executable='explorer',      # Corrected executable name based on screenshot
            name='explorer_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        
        # Thermal Node
        Node(
            package='cde2310',  # Corrected package name based on screenshot
            executable='thermal',      # Corrected executable name based on screenshot
            name='thermal',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
    ])

