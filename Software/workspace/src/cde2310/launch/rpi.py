import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    tb3_bringup_dir = get_package_share_directory('turtlebot3_bringup')
    cde2310_dir = get_package_share_directory('cde2310')
    custom_explorer_dir = get_package_share_directory('custom_explorer')

    return LaunchDescription([
        # Robot bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_bringup_dir, 'launch', 'robot.launch.py')
            )
        ),
       
        TimerAction(
            period=10.0,  # delay in seconds
            actions=[
                Node(
                    package='custom_explorer',
                    executable='explorer',
                    name='explorer_node',
                    output='screen',
                    parameters=[{'use_sim_time': False}]
                ),
                # Thermal node
                Node(
                    package='cde2310',
                    executable='thermal',
                    name='thermal',
                    output='screen',
                    parameters=[{'use_sim_time': False}]
                ),
            ]
        )
    ])

