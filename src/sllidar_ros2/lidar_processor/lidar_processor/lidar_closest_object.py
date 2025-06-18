import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Float32MultiArray
from std_msgs.msg import Int8
import numpy as np

class FilteredLidarNode(Node):
    def __init__(self):
        super().__init__('filtered_lidar')
        self.pub = 0
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
        #self.filtered_pub = self.create_publisher(Float32MultiArray, 'filtered_scan', 10)
        self.filtered_pub =self.create_publisher(Int8, 'filtered_scan', 10)

        # Store latest filtered data and rotation angle
        self.latest_filtered_data = Float32MultiArray()
        self.rotation_pot = 0.0  # Default value
        self.length = 10.0  # Default ground length

        # Timer for publishing at a higher rate (20 Hz)
        self.timer = self.create_timer(0.05, self.publish_filtered_data)


    def rotation_callback(self, msg):
        """ Updates the latest rotation potentiometer value. """
        self.rotation_pot = -msg.data
        #self.get_logger().info(f"Rotation potentiometer value: {self.rotation_pot}")
        self.Bl = 180 - (90 + self.rotation_pot)
        self.cl = 11.5
        self.pot_rads = np.deg2rad(self.rotation_pot)
        self.a = np.sin(self.pot_rads) * self.cl
        self.bl = np.cos(self.pot_rads) * self.cl
        self.bs = self.length - self.bl
        self.cs = np.sqrt(self.a**2 + self.bs**2)
        self.Bs_rads = np.arccos(self.a / self.cs)
        self.Bs = np.rad2deg(self.Bs_rads)
        self.B_combined = self.Bs + self.Bl
        self.vinkel = 180 - self.B_combined
        #self.get_logger().info(f"B combined: {self.B_combined}")

    def length_callback(self, msg):
        """ Updates the ground length value. """
        self.length = msg.data
        #self.get_logger().info(f"Ground length value: {self.length}")
        #print(f"Received ground length: {msg.data}")


    def scan_callback(self, msg):
       """ Processes LiDAR data and applies filtering based on rotation_pot. """
       #self.get_logger().info("Før try")
       try:
           ranges = np.array(msg.ranges, dtype=np.float32)
           angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges), dtype=np.float32)
           #self.get_logger().info(f"Angles test: {np.degrees(angles)}")

           # Filter based on rotation potentiometer value and ground length
           valid_mask = (angles >= np.radians(-7 + self.vinkel - self.rotation_pot)) & \
                     (angles <= np.radians(7 + self.vinkel - self.rotation_pot)) & \
                     np.isfinite(ranges) & \
                     (ranges <= self.length - 0.5)
           #self.get_logger().info("{valid_mask}")
           if np.any(valid_mask):
               filtered_angles = np.degrees(angles[valid_mask])
               filtered_distances = ranges[valid_mask]

            # Store latest filtered data
               self.latest_filtered_data.data = np.column_stack((filtered_angles, filtered_distances)).flatten().tolist()
               #self.get_logger().warn("Valid points exist")
               mean_angle = np.mean(filtered_angles)
               #self.get_logger().info(f"Mean filtered angle: {mean_angle:.2f} degrees")

               if mean_angle < (self.vinkel - self.rotation_pot):
                   self.pub = 1
                   self.get_logger().info(f"Left")

               else:
                   self.pub = 2
                   self.get_logger().info(f"Right")

               #self.get_logger().info(f"Filtered points: {np.sum(valid_mask)}")

           else:
               #self.get_logger().warn("No valid LiDAR points found in filter range")
               self.pub = 0
               self.latest_filtered_data.data = []
       except Exception as e:
           #self.get_logger().error(f"Error processing LiDAR data: {e}")
           pass
    def publish_filtered_data(self):
        """ Publishes the latest filtered scan data. """
        #self.get_logger().info("Publishing")
        msg = Int8()
        msg.data = self.pub
        self.filtered_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FilteredLidarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
