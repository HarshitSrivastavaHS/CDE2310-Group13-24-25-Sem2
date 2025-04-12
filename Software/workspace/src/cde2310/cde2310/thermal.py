import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import board
import busio
import adafruit_amg88xx

class ThermalPublisher(Node):
    def __init__(self):
        super().__init__('thermal_publisher')
        
        # Initialize I2C bus and sensor
        i2c_bus = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_amg88xx.AMG88XX(i2c_bus)
        
        # ROS Publisher
        self.publisher_ = self.create_publisher(String, 'thermal_data', 10)
        
        # Timer to publish data every 1 second
        self.timer = self.create_timer(1.0, self.publish_thermal_data)
        
        self.get_logger().info("Thermal Publisher Node Started")

    def publish_thermal_data(self):
        # Analyze the sensor data and decide on direction
        direction = self.analyze_sensor_data()
        
        # Prepare message to be published
        msg = String()
        msg.data = direction
        
        # Publish the message
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published Thermal Data: {msg.data}")

    def analyze_sensor_data(self):
        # Read the sensor data (8x8 array)
        sensor_data = self.sensor.pixels
        self.get_logger().info(f"{sensor_data}")
        # Set threshold for detecting high dots
        threshold = 30.0  # Adjust this value based on the sensor's sensitivity

        # Count the number of pixels above threshold
        above_threshold = sum(1 for row in sensor_data for pixel in row if pixel > threshold)
        
        # Calculate number of high dots on the left, right, and center
        total_pixels = len(sensor_data) * len(sensor_data[0])
        # Check if all are below the threshold
        if above_threshold < 0.05*total_pixels:
            return "N"  # All dots are below threshold
        
        # Check if more than 70% of dots are above threshold
        if above_threshold > 0.7 * total_pixels:
            return "S"  # More than 70% are above threshold
        
        left_count = sum(1 for row in sensor_data for i in range(4) if row[i] > threshold)  # Left half
        right_count = sum(1 for row in sensor_data for i in range(4, 8) if row[i] > threshold)  # Right half
        center_count = sum(1 for row in sensor_data for i in range(2, 6) if row[i] > threshold)  # Center
        
        # Decide the direction based on high dot positions
        if center_count > left_count and center_count > right_count:
           return f"F{center_count}"
        elif left_count > right_count:
           return "L"
        else:
           return "R"
           
        """
        if left_count > right_count and left_count > center_count:
            return "L"  # More high dots on the left
        elif right_count > left_count and right_count > center_count:
            return "R"  # More high dots on the right
        else:
            return "F"+str(center_count)  # Otherwise, assume it's going straight
        """

def main(args=None):
    rclpy.init(args=args)
    node = ThermalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Thermal Publisher")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
