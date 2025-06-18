import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Float32MultiArray
import numpy as np

class FilteredLidarNode(Node):
    def __init__(self):
        super().__init__('filtered_lidar')

        # Subscriber for LiDAR scan
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            100)

        # Subscriber for rotation potentiometer
        self.rotation_sub = self.create_subscription(
            Float32, 'rotation_pot', self.rotation_callback, 10)

        # Subscriber for ground length
        self.length_sub = self.create_subscription(
            Float32, 'ground_length', self.length_callback, 10)

        # Publisher for filtered scan data
        self.filtered_pub = self.create_publisher(Float32MultiArray, 'filtered_scan', 10)

        # Store latest filtered data and rotation angle
        self.latest_filtered_data = Float32MultiArray()
        self.rotation_pot = 0.0  # Default value
        self.length = 10.0  # Default ground length

        # Timer for publishing at a higher rate (20 Hz)
        self.timer = self.create_timer(0.05, self.publish_filtered_data)

    def rotation_callback(self, msg):
        """ Updates the latest rotation potentiometer value. """
        self.rotation_pot = msg.data
        self.get_logger().info(f"Rotation potentiometer value: {self.rotation_pot}")

    def length_callback(self, msg):
        """ Updates the ground length value. """
        self.length = msg.data
        #self.get_logger().info(f"Ground length value: {self.length}")
        #print(f"Received ground length: {msg.data}")

    def scan_callback(self, msg):
        """ Processes LiDAR data and applies filtering based on rotation_pot. """
        try:
            ranges = np.array(msg.ranges, dtype=np.float32)
            angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges), dtype=np.float32)

            # Filter based on rotation potentiometer value and ground length
            valid_mask = (angles >= np.radians(-5 + self.rotation_pot)) & \
                         (angles <= np.radians(5 + self.rotation_pot)) & \
                         np.isfinite(ranges) & \
                         (ranges <= self.length - 0.5)

            if np.any(valid_mask):
                filtered_angles = np.degrees(angles[valid_mask])
                filtered_distances = ranges[valid_mask]

                # Store latest filtered data
                self.latest_filtered_data.data = np.column_stack((filtered_angles, filtered_distances)).flatten().tolist()
            else:
                self.get_logger().warn("No valid points found")

        except Exception as e:
            self.get_logger().error(f"Error processing LiDAR data: {e}")

    def publish_filtered_data(self):
        """ Publishes the latest filtered scan data at a fixed rate. """
        if self.latest_filtered_data.data:
            self.filtered_pub.publish(self.latest_filtered_data)

def main(args=None):
    rclpy.init(args=args)
    node = FilteredLidarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
