import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import numpy as np

class FakeThermalPublisher(Node):
    def __init__(self):
        super().__init__('thermal')
        self.publisher_ = self.create_publisher(Int32, 'thermal_data', 10)
        self.timer = self.create_timer(1.0, self.publish_fake_data)
        self.get_logger().info("Fake Thermal Publisher Started")
        
        self.thermal_data = 5  # Single integer value

    def publish_fake_data(self):
        """Publishes a single integer value"""
        msg = Int32()
        msg.data = self.thermal_data

        self.publisher_.publish(msg)
        self.get_logger().info(f"Published Fake Thermal Data: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = FakeThermalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Fake Thermal Publisher")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
