import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import Int32
import numpy as np
import math


class ExplorerNode(Node):
    def __init__(self):
        super().__init__('explorer')
        self.get_logger().info("Explorer Node Started")

        # Subscriber to the map topic
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.thermal_sub = self.create_subscription(Int32, '/thermal_data', self.thermal_callback, 10)  #thermal data

        # Action client for navigation
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Visited frontiers set
        self.visited_frontiers = set()
        self.visited_heat_sources = []
        
        self.followingHeat = None
                
        self.heat_source = None  # Latest detected heat location
        self.heat_threshold = 0.5  # Min distance to avoid revisiting (meters)

        # Map and position data
        self.map_data = None
        self.robot_position = (0, 0)  # Placeholder, update from localization

        # Timer for periodic exploration
        self.timer = self.create_timer(5.0, self.check_heat_source)

    def map_callback(self, msg):
        self.map_data = msg
        self.get_logger().info("Map received")
        
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
        self.heat_source = 5

    def check_heat_source(self):
        """
        Move in the direction of the detected heat source. If no heat source is detected, explore.
        """
        if self.heat_source:
            self.get_logger().info("YAY")
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
        else:
            # No heat source detected, continue exploring
            self.get_logger().info("No heat source detected, continuing exploration.")
            self.explore()




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
            return

        self.get_logger().info("Goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_complete_callback)

    def navigation_complete_callback(self, future):
        """
        Callback to handle the result of the navigation action.
        """
        try:
            result = future.result().result
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
        explorer_node.destroy_node()
        rclpy.shutdown()
