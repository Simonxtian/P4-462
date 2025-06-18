import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time

class FilteredLidarSubscriber(Node):
    def __init__(self):
        super().__init__('filtered_lidar_subscriber')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'filtered_scan',
            self.scan_callback,
            1000
        )

        self.timer = self.create_timer(1.0, self.check_no_message)
        self.last_received_time = time.time()
        self.previous_msg = None
        self.get_logger().info("FilteredLidarSubscriber node started.")

    def scan_callback(self, msg):
        if msg.data == self.previous_msg:
            # It's a duplicate message — don't update the time
            self.get_logger().info("Duplicate message received, ignoring.")
            return

        # It's a new message
        self.last_received_time = time.time()
        self.get_logger().info(f"New message received: {msg.data}")
        self.previous_msg = msg.data

    def check_no_message(self):
        if time.time() - self.last_received_time > 1.0:
            self.get_logger().warn("No NEW messages received for the last 1 second")

def main(args=None):
    rclpy.init(args=args)
    node = FilteredLidarSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
