import subprocess
import time
def launch_ros_commands():
    commands = [
        "ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py",
        "ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz",
    ]
    
    for cmd in commands:
        subprocess.Popen(["gnome-terminal", "--", "bash", "-c", f"sleep 2; {cmd}; exec bash"])
        time.sleep(3);

if __name__ == "__main__":
    launch_ros_commands()

