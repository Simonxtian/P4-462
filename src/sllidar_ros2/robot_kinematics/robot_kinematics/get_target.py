import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int8
import numpy as np

class PlankTracker(Node):
    def __init__(self):
        super().__init__('plank_tracker')

        self.x_planke = 0.0
        self.y_planke = 0.0
        self.x_r = 0.0
        self.y_r = 0.0
        self.phi = 0.0

        self.prev_gamma = 0.0
        self.last_time = self.get_clock().now()

        self.obstacle = 0

        # Subscribers
        self.create_subscription(Float32MultiArray, 'robot_pose', self.robot_pose_callback, 10)
        self.create_subscription(Float32MultiArray, 'plank_position', self.plank_position_callback, 10)
        self.create_subscription(Int8, 'filtered_scan', self.lidar_callback, 10)

        # Publisher
        self.tracking_publisher = self.create_publisher(Float32MultiArray, 'plank_tracking', 10)

        # Timer loop
        self.timer = self.create_timer(0.05, self.update)

    def robot_pose_callback(self, msg):
        if len(msg.data) == 3:
            self.x_r, self.y_r, self.phi = msg.data

    def plank_position_callback(self, msg):
        if len(msg.data) == 2:
            self.x_planke, self.y_planke = msg.data

    def lidar_callback(self, msg):
        self.obstacle = msg.data

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        gamma, x_t, y_t = self.get_angle_error()
        omega = (gamma - self.prev_gamma) / dt if dt > 0 else 0.0
        self.prev_gamma = gamma

        # Log and publish
        #self.get_logger().info(f"gamma: {gamma:.4f}, omega: {omega:.4f}, x_t: {x_t:.2f}, y_t: {y_t:.2f}")

        msg = Float32MultiArray()
        msg.data = [x_t, y_t, gamma, omega]
        self.tracking_publisher.publish(msg)

    def get_angle_error(self):
        # Robot direction vector
        R = np.array([np.cos(self.phi), np.sin(self.phi)])

        if self.obstacle != 0:
            if self.obstacle == 1:
                self.y_planke += 0.2
                self.get_logger().info(f"Moving point ({self.x_planke:.2f}, {self.y_planke:.2f}) to left")

            if self.obstacle == 2:
                self.y_planke -= 0.2
                self.get_logger().info(f"Moving point ({self.x_planke:.2f}, {self.y_planke:.2f}) to right")

        # Rotate local plank position to global
        rotation_matrix = np.array([
            [np.cos(self.phi), -np.sin(self.phi)],
            [np.sin(self.phi),  np.cos(self.phi)]
        ])
        plank_global = np.dot(rotation_matrix, [self.x_planke, self.y_planke])

        x_t = plank_global[0] + self.x_r
        y_t = plank_global[1] + self.y_r

        T = np.array([x_t - self.x_r, y_t - self.y_r])

        R_norm = R / np.linalg.norm(R)
        T_norm = T / np.linalg.norm(T)
        gamma = np.arccos(np.clip(np.dot(R_norm, T_norm), -1.0, 1.0))

        cross = np.cross(R_norm, T_norm)
        if cross < 0:
            gamma = -gamma  # preserve sign

        return gamma, x_t, y_t

def main(args=None):
    rclpy.init(args=args)
    node = PlankTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
