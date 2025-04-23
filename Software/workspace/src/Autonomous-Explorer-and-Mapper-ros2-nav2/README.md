**Explorer Node Overview**

This code implements a ROS2 (Robot Operating System) node that enables a robot to autonomously explore its environment using a thermal sensor (simulated here) and navigate based on detected heat sources. It subscribes to various topics such as the map, odometry, and thermal sensor data, and publishes commands to control the robot's movement.

**Key Functionalities:**

1. **Map and Odometry Handling:**
    - Subscribes to /map for the robot’s environment map (OccupancyGrid).
    - Subscribes to /odom for the robot's current position and orientation (Odometry).
2. **Thermal Sensor Integration:**
    - Subscribes to /thermal_data for thermal sensor readings.
    - Identifies heat sources and makes decisions based on their locations.
3. **Autonomous Navigation:**
    - Uses an action client (NavigateToPose) to navigate to specific positions.
    - Based on thermal data, the robot moves towards detected heat sources, avoids revisiting them, and can perform specific tasks like firing a projectile at heat sources.

**Important Methods:**

**1\. map_callback(msg)**

- **Purpose:** This method receives the environment map (OccupancyGrid) and stores it in the map_data attribute. The map is used later for navigation and exploration.

**2\. odom_callback(msg)**

- **Purpose:** This method processes the robot's position and orientation (odometry data). It extracts the robot's current coordinates and orientation (in radians) using a quaternion-to-Euler conversion.

**3\. thermal_callback(msg)**

- **Purpose:** Receives thermal data from the thermal sensor. The sensor data would be used to identify the location of heat sources. 

**4\. check_heat_source()**

- **Purpose:** This method is periodically triggered by a timer. It checks if a heat source has been detected:
  - If no heat source is detected, it calls self.explore() to continue searching.
  - If a heat source is found, it evaluates the type of action needed based on the heat source's characteristics (e.g., move forward, turn, or fire a projectile).

**5\. startFiring()**

- **Purpose:** This function controls a PWM pin on a Raspberry Pi to simulate firing a flare (a ping pong ball) when a heat source is detected. The firing time is controlled by the PWM_PIN and DUTY_CYCLE parameters.

**6\. move_forward(pixels)**

- **Purpose:** This function computes the target coordinates (target_x, target_y) based on the current robot position and the desired forward distance (calculated from thermal data). It then sends a navigation goal to move the robot towards those coordinates using the NavigateToPose action.

**7\. turn_left_in_place() and turn_right_in_place()**

- **Purpose:** These methods control the robot’s in-place rotation using the cmd_vel topic. The robot turns a specified number of degrees (default 10°) either to the left or right by publishing angular velocity commands.

**Other Key Features:**

- **PWM Control for Firing Mechanism:** The startFiring method demonstrates controlling a firing mechanism via PWM on the Raspberry Pi's GPIO pin. This action is triggered when a specific type of heat source is detected.
- **Exploration and Heat Source Handling:** The robot follows basic rules for detecting and responding to heat sources:
  - If the heat source is a "L" (left turn), it will turn left.
  - If it's "R" (right turn), the robot turns right.
  - If it's "F" (forward), the robot moves forward towards the heat source if it hasn't already visited it.
  - If the heat source is "S" (fire), it activates the firing mechanism.

**Navigation Control:**

- The NavigateToPose action client is used to send movement goals to the robot, either for exploration or for specific heat source targets. These navigation goals are based on the robot’s current position and orientation.

**Dependencies:**

- **ROS2**: This node uses ROS2 for handling topics, actions, and messages (nav_msgs, geometry_msgs, std_msgs, etc.).
- **RPi.GPIO**: For controlling the Raspberry Pi's GPIO pins to manage the firing mechanism.
- **NumPy**: For handling thermal data as arrays.
- **tf_transformations**: For converting quaternion-based orientation data to Euler angles (and vice versa).

---

### The frontier based exploration is forked from a different repo. Credits to the original owner: [AniArka - Autonomous Explorer and Mapper Ros2 Nav2](https://github.com/AniArka/Autonomous-Explorer-and-Mapper-ros2-nav2)



