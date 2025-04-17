import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import String
import numpy as np
import math
import tf_transformations
import RPi.GPIO as GPIO
import time
from rclpy.duration import Duration


class ExplorerNode(Node):
    def __init__(self):
        super().__init__('explorer')
        self.get_logger().info("Explorer Node Started")

        # Subscriber to the map topic
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.thermal_sub = self.create_subscription(String, '/thermal_data', self.thermal_callback, 10)  #thermal data
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.current_orientation = None
        # Action client for navigation
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Visited frontiers set
        self.visited_frontiers = set()
        self.visited_heat_sources = []
        
        self.followingHeat = None
        self.exploring = False

        self.current_goal_handle = None

        GPIO.setmode(GPIO.BCM)

        # Set GPIO pin 18 as output
        GPIO.setup(18, GPIO.OUT)
        GPIO.setup(17, GPIO.OUT)
        GPIO.setup(27, GPIO.OUT)
        GPIO.output(17, GPIO.HIGH)
        GPIO.output(27, GPIO.LOW)
                
        self.heat_source = None  # Latest detected heat location
        self.heat_threshold = 0.5  # Min distance to avoid revisiting (meters)

        # Map and position data
        self.map_data = None
        self.robot_position = (0, 0)  # Placeholder, update from localization

        # Timer for periodic exploration
        self.timer = self.create_timer(1.0, self.check_heat_source)

    def map_callback(self, msg):
        self.map_data = msg
        self.get_logger().info("Map received")

    def odom_callback(self, msg):
        # Extract quaternion from odometry
        position = msg.pose.pose.position
        self.robot_position = (position.x, position.y)

        orientation_q = msg.pose.pose.orientation
        quat = (orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
        roll, pitch, self.current_yaw = tf_transformations.euler_from_quaternion(quat)
        
    def thermal_callback(self, msg):
        """
        Simulated thermal data callback. In real use, this would process AMG8833 data.
        
        thermal_array = np.array(msg.data).reshape((8, 8))  # 8x8 thermal grid
        max_temp = np.max(thermal_array)

        if max_temp > 30:  # Arbitrary threshold for heat detection
            heat_idx = np.unravel_index(np.argmax(thermal_array), thermal_array.shape)
            self.heat_source = heat_idx
            self.get_logger().info(f"Detected heat source at {heat_idx}, Temp={max_temp}")
        else:
            self.heat_source = None
        """
        self.heat_source = msg.data

    def startFiring(self):
        GPIO.output(18, GPIO.HIGH)
        self.get_clock().sleep_for(Duration(seconds=2))
        GPIO.output(18, GPIO.LOW)
        self.get_clock().sleep_for(Duration(seconds=2))
        GPIO.output(18, GPIO.HIGH)
        self.get_clock().sleep_for(Duration(seconds=4))
        GPIO.output(18, GPIO.LOW)
        

    def check_heat_source(self):
        """
        Move in the direction of the detected heat source. If no heat source is detected, explore.
        """
        if self.heat_source is None:
            self.get_logger().info("Heat source is empty.")
            self.explore()
            return


        self.get_logger().info(f"{self.heat_source}")
        if self.map_data is None:
            self.get_logger().warning("No map data available")
            return
        
        if self.heat_source[0] == "N":
            self.get_logger().info("No heat source detected, continuing exploration.")
            self.explore()
            return
        
        if self.exploring:
            self.current_goal_handle.cancel_goal_async()
            self.current_goal_handle = None
            self.exploring = False

        match(self.heat_source[0]):
            
            case "L":
                self.left_turn()
            case "R":
                self.right_turn()
            case "F":
                x, y = self.getCoordinates(self.heat_source[1:])
                result = any(math.sqrt((x - x1)**2 + (y - y1)**2) < 0.15 for (x1, y1) in self.visited_heat_sources) 
                if result:
                    self.get_logger().info("Already visited heat source")
                    self.explore()
                else:
                    self.move_forward(self.heat_source[1:])
                
            case "S":
                """
                FIRE THE PING PONG BALLS
                """
                self.startFiring()
                self.visited_heat_sources.append(self.robot_position)
            

        """heat_row, heat_col = self.heat_source  # Assuming (row, col) format
            heat_row, heat_col = heat_row - 4, heat_col - 4
            # self.get_logger().info(f"LOL:{heat_col}")
            
            angle_rad = ((heat_col + 4) * (30 / 7) - 30) * 2 * 3.1412 / 360
            w = math.cos(angle_rad / 2)
            z = math.sin(angle_rad / 2)
            
            goal_msg = PoseStamped()
            goal_msg.header.frame_id = 'map'
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.position.x = self.map_data.info.origin.position.x
            goal_msg.pose.position.y = self.map_data.info.origin.position.y
            goal_msg.pose.orientation.w = w
            goal_msg.pose.orientation.z = z
            
            self.get_logger().info(f"{heat_row}, {heat_col}")
            # Wait for the action server
            nav_goal = NavigateToPose.Goal()
            nav_goal.pose = goal_msg

            self.nav_to_pose_client.wait_for_server()

            # Send the goal and register a callback for the result
            send_goal_future = self.nav_to_pose_client.send_goal_async(nav_goal)
            send_goal_future.add_done_callback(self.goal_response_callback)
        """
        
        
    def getForwardDistance(self, pixels):
        dist = 0
        if pixels > 32:
            dist = 0.0
        elif pixels > 8:
            dist = 0.2
        else:
            dist = 0.45
        return  dist

    def getCoordinates(self, pixels):
        robot_x, robot_y = self.robot_position  # Update this from your odometry data
        current_yaw = self.current_yaw  # This is the yaw angle in radians
    
        # Set the distance to move forward (e.g., 1 meter)
        forward_distance = self.getForwardDistance(int(pixels))
    
        # Calculate the target position in front of the robot
        target_x = robot_x + forward_distance * math.cos(current_yaw)
        target_y = robot_y + forward_distance * math.sin(current_yaw)
        return (target_x, target_y)

    def move_forward(self, pixels):
        target_x, target_y = self.getCoordinates(pixels)
    
        # Prepare the goal message
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = target_x
        goal_msg.pose.position.y = target_y
    
        # Calculate the orientation (quaternion) to face forward
        q = tf_transformations.quaternion_from_euler(0, 0, self.current_yaw)
        goal_msg.pose.orientation.x = q[0]
        goal_msg.pose.orientation.y = q[1]
        goal_msg.pose.orientation.z = q[2]
        goal_msg.pose.orientation.w = q[3]
    
        # Send the navigation goal
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_msg
    
        self.get_logger().info(f"Navigating to forward position: x={target_x}, y={target_y}")
    
        # Wait for the action server
        self.nav_to_pose_client.wait_for_server()
    
        # Send the goal and register a callback for the result
        send_goal_future = self.nav_to_pose_client.send_goal_async(nav_goal)
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def right_turn(self):
        """
        Turn the robot 90 degrees to the right.
        """
        current_yaw = self.current_yaw
        new_yaw = current_yaw - 4 * math.pi   # Right turn is a -90 degree rotation

        # Normalize the yaw angle to stay within [-pi, pi]
        new_yaw = (new_yaw + math.pi) % (2 * math.pi) - math.pi

        # Prepare the goal message
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = float(self.robot_position[0])  # Keep the same position
        goal_msg.pose.position.y = float(self.robot_position[1])  # Keep the same position

        # Calculate the orientation (quaternion) for the new yaw
        q = tf_transformations.quaternion_from_euler(0, 0, new_yaw)
        goal_msg.pose.orientation.x = q[0]
        goal_msg.pose.orientation.y = q[1]
        goal_msg.pose.orientation.z = q[2]
        goal_msg.pose.orientation.w = q[3]

        # Send the navigation goal
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_msg

        self.get_logger().info(f"Turning right to new orientation: yaw={new_yaw}")

        # Wait for the action server
        self.nav_to_pose_client.wait_for_server()

        # Send the goal and register a callback for the result
        send_goal_future = self.nav_to_pose_client.send_goal_async(nav_goal)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def left_turn(self):
        """
        Turn the robot 90 degrees to the right.
        """
        current_yaw = self.current_yaw
        new_yaw = current_yaw + math.pi / 2  # Right turn is a -90 degree rotation

        # Normalize the yaw angle to stay within [-pi, pi]
        new_yaw = (new_yaw + math.pi) % (2 * math.pi) - math.pi

        # Prepare the goal message
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = float(self.robot_position[0])  # Keep the same position
        goal_msg.pose.position.y = float(self.robot_position[1])  # Keep the same position

        # Calculate the orientation (quaternion) for the new yaw
        q = tf_transformations.quaternion_from_euler(0, 0, new_yaw)
        goal_msg.pose.orientation.x = q[0]
        goal_msg.pose.orientation.y = q[1]
        goal_msg.pose.orientation.z = q[2]
        goal_msg.pose.orientation.w = q[3]

        # Send the navigation goal
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_msg

        self.get_logger().info(f"Turning right to new orientation: yaw={new_yaw}")

        # Wait for the action server
        self.nav_to_pose_client.wait_for_server()

        # Send the goal and register a callback for the result
        send_goal_future = self.nav_to_pose_client.send_goal_async(nav_goal)
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def navigate_to(self, x, y):
        """
        Send navigation goal to Nav2.
        """
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.orientation.w = 1.0  # Facing forward

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_msg

        self.get_logger().info(f"Navigating to goal: x={x}, y={y}")

        # Wait for the action server
        self.nav_to_pose_client.wait_for_server()

        self.exploring = True

        # Send the goal and register a callback for the result
        send_goal_future = self.nav_to_pose_client.send_goal_async(nav_goal)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        Handle the goal response and attach a callback to the result.
        """
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warning("Goal rejected!")
            self.exploring = False
            return

        self.get_logger().info("Goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_complete_callback)
        
        self.current_goal_handle = goal_handle

    def navigation_complete_callback(self, future):
        """
        Callback to handle the result of the navigation action.
        """
        try:
            result = future.result().result
            if self.exploring:
                self.exploring = False
            if (self.followingHeat and result):
            	self.visited_heat_sources.append(self.followingHeat)
            	self.followingHeat = None
            self.get_logger().info(f"Navigation completed with result: {result}")
        except Exception as e:
            self.get_logger().error(f"Navigation failed: {e}")

    def find_frontiers(self, map_array):
        """
        Detect frontiers in the occupancy grid map.
        """
        frontiers = []
        rows, cols = map_array.shape

        # Iterate through each cell in the map
        for r in range(1, rows - 1):
            for c in range(1, cols - 1):
                if map_array[r, c] == 0:  # Free cell
                    # Check if any neighbors are unknown
                    neighbors = map_array[r-1:r+2, c-1:c+2].flatten()
                    if -1 in neighbors:
                        frontiers.append((r, c))

        self.get_logger().info(f"Found {len(frontiers)} frontiers")
        return frontiers

    def choose_frontier(self, frontiers):
        """
        Choose the closest frontier to the robot.
        """
        robot_row, robot_col = self.robot_position
        min_distance = float('inf')
        chosen_frontier = None
        

        for frontier in frontiers:
            if frontier in self.visited_frontiers:
                continue

            distance = np.sqrt((robot_row - frontier[0])**2 + (robot_col - frontier[1])**2)
            if distance < min_distance:
                min_distance = distance
                chosen_frontier = frontier

        if chosen_frontier:
            self.visited_frontiers.add(chosen_frontier)
            self.get_logger().info(f"Chosen frontier: {chosen_frontier}")
        else:
            self.get_logger().warning("No valid frontier found")

        return chosen_frontier

    def explore(self):
        if self.map_data is None:
            self.get_logger().warning("No map data available")
            return
        
        if self.exploring:
            return

        # Convert map to numpy array
        map_array = np.array(self.map_data.data).reshape(
            (self.map_data.info.height, self.map_data.info.width))

        # Detect frontiers
        frontiers = self.find_frontiers(map_array)
        
        if not frontiers:
            self.get_logger().info("No frontiers found. Exploration complete!")

            # self.shutdown_robot()
            return

        # Choose the closest frontier
        chosen_frontier = self.choose_frontier(frontiers)

        if not chosen_frontier:
            self.get_logger().warning("No frontiers to explore")
            # self.get_logger().info(frontiers)
            return

        # Convert the chosen frontier to world coordinates
        goal_x = chosen_frontier[1] * self.map_data.info.resolution + self.map_data.info.origin.position.x
        goal_y = chosen_frontier[0] * self.map_data.info.resolution + self.map_data.info.origin.position.y

        # Navigate to the chosen frontier
        self.navigate_to(goal_x, goal_y)

    # def shudown_robot(self):
    #     
    #
    #
    #     self.get_logger().info("Shutting down robot exploration")


def main(args=None):
    rclpy.init(args=args)
    explorer_node = ExplorerNode()

    try:
        explorer_node.get_logger().info("Starting exploration...")
        rclpy.spin(explorer_node)
    except KeyboardInterrupt:
        explorer_node.get_logger().info("Exploration stopped by user")
    finally:
        GPIO.cleanup()
        explorer_node.destroy_node()
        rclpy.shutdown()
