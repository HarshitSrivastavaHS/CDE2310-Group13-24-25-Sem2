# 🧠 Software 

This directory contains all software components used in the robot that autonomously explores an environment, detects thermal heat signatures (simulating trapped humans), and fires a flare to mark the location.

The software runs on **Ubuntu 22.04 (64-bit)** with **ROS2 Humble**, deployed on a **Raspberry Pi 4 (4GB RAM)**.

---

## ⚙️ Setup Instructions

These steps will help you replicate the software environment on a fresh Raspberry Pi setup.

### 1. Clone the Repository

```
git clone https://github.com/<your-username>/CDE2310-Group13-24-25-Sem2.git
cd CDE2310-Group13-24-25-Sem2/Software
```

### 2. Install ROS2 Humble

Follow the official guide for ROS2 Humble on Ubuntu 22.04:  
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

After installing, add ROS to your shell:

```
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3. Install Required Libraries

You’ll need the following Python libraries on the Raspberry Pi:

```
pip install adafruit-circuitpython-amg88xx
pip install numpy
```

Also make sure `RPi.GPIO` is available (comes preinstalled on most RPi OSes):

```
sudo apt install python3-rpi.gpio
```

Make sure the following ROS2 packages are installed:

```
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-slam-toolbox
```

### 4. Build the ROS2 Workspace

```
cd workspace
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

---

## 🧪 Nodes Overview

- **thermal.py**  
  Detects heat sources using the AMG8833 sensor and publishes direction commands (`L`, `R`, `F<number>`, `S`) on `/thermal_direction`.

- **explorer.py**  
  Custom frontier-based exploration node. Uses SLAM map to detect unexplored frontiers and plans paths using Nav2 + Dijkstra. Listens to `/thermal` to decide when to stop and fire. Also controls the GPIO pins on the Raspberry Pi to trigger the firing mechanism (flare shooter).

All nodes are written in Python using the `rclpy` client library and structured as standalone ROS2 nodes.

---

## 📂 Folder Structure

- `workspace/` — ROS2 workspace (nodes live in `src/`)
- Other Files — Some Test files written during development and testing.

---

## 🧷 Useful Aliases

You can set up these in your `~/.bashrc` to make launching components easier:

```
alias config="cd ~/CDE2310-Group13-24-25-Sem2/Software/workspace/src/cde2310/config"
alias nav2="config && ros2 launch nav2_bringup navigation_launch.py params_file:=./nav2_params.yaml"
alias slam="ros2 launch slam_toolbox online_async_launch.py"
alias thermal="ros2 run cde2310 thermal"
alias explore="ros2 run custom_explorer explorer"
```

---

## 🧯 System Behavior

1. Robot starts SLAM and begins exploring via the `explorer` node.
2. On detecting a heat signature, `thermal` node sends direction codes to `explorer`.
3. `explorer` aligns the robot toward the heat and moves forward until detection is maximized.
4. When the heat is confirmed (`S`), it triggers the launcher using GPIO and marks the location.
5. It then resumes exploration and ignores previously visited heat spots.

---

## 🛠 Testing Notes

Each module was individually tested and passed. However, due to time constraints, final integrated behavior had issues:

- The robot froze during continuous navigation updates when multiple `/thermal_direction` messages were received.
- Arbitration logic between thermal and exploration was insufficient.
- Final demo attempt failed due to lack of smooth integration (our bad 😔).

Future improvements would include using a centralized state machine or ROS2 behavior trees to manage transitions between exploring, detecting, aligning, and firing.

---
