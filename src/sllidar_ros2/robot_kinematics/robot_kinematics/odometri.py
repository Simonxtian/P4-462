import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32MultiArray
import numpy as np

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        # Robot parameters
        self.WHEEL_RADIUS = 0.10  # meters
        self.WHEEL_BASE = 0.405   # meters

        # Initial robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_l_tick = None
        self.last_r_tick = None
        self.l_tick = 0
        self.r_tick = 0

        # Subscriber to encoder counts
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'encoder',
            self.encoder_callback,
            10)

        # Publisher for pose
        self.pose_publisher = self.create_publisher(Float32MultiArray, 'robot_pose', 10)

        self.timer = self.create_timer(0.05, self.calculate_and_publish)

    def encoder_callback(self, msg: Float32MultiArray):
        if len(msg.data) != 4:
            self.get_logger().warn('Received encoder message with incorrect length.')
            return

        self.l_tick, self.r_tick, _, _ = msg.data

    def calculate_and_publish(self):
        # Initialize last tick on first message
        if self.last_l_tick is None or self.last_r_tick is None:
            self.last_l_tick = self.l_tick
            self.last_r_tick = self.r_tick
            return

        delta_l = self.l_tick - self.last_l_tick
        delta_r = self.r_tick - self.last_r_tick
        self.last_l_tick = self.l_tick
        self.last_r_tick = self.r_tick

        distance_per_rev = (2 * np.pi * self.WHEEL_RADIUS)
        d_L = delta_l * distance_per_rev
        d_R = delta_r * distance_per_rev

        d = (d_L + d_R) / 2
        d_theta = (d_R - d_L) / self.WHEEL_BASE

        # Update robot position
        self.x += d * np.cos(self.theta + d_theta / 2)
        self.y += d * np.sin(self.theta + d_theta / 2)
        self.theta += d_theta
        self.theta = (self.theta + np.pi) % (2 * np.pi) - np.pi

        # Publish pose
        pose_msg = Float32MultiArray()
        pose_msg.data = [float(self.x), -float(self.y), -float(self.theta)]
        self.pose_publisher.publish(pose_msg)

        self.get_logger().info(f"Pose: x={self.x:.2f}, y={self.y:.2f}, θ={self.theta:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
