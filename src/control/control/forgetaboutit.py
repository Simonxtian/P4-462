import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
import numpy as np

class PIController:
    def __init__(self, kp, ki, dt, output_limits=(-2.5, 2.5)):
        self.kp = kp
        self.ki = ki
        self.dt = dt
        self.integral = 0.0
        self.output_limits = output_limits

    def reset(self):
        self.integral = 0.0

    def compute(self, error):
        self.integral += error * self.dt
        output = self.kp * error + self.ki * self.integral
        min_out, max_out = self.output_limits
        return max(min_out, min(max_out, output))

class AngularErrorCorrectionNode(Node):
    def __init__(self):
        super().__init__('angular_error_correction_node')

        self.dt = 0.1  # Control loop time step
        self.pi_controller = PIController(kp=1.5, ki=0.0, dt=self.dt, output_limits=(-2.5, 2.5))

        self.angular_error = None
        self.coord_list_x = []
        self.coord_list_y = []
        self.current_pose = None  # Will store (x, y, theta)

        # Subscriptions
        self.create_subscription(Float32MultiArray, 'plank_tracking', self.plank_callback, 10)
        self.create_subscription(Float32MultiArray, 'robot_pose', self.current_pose_callback, 10)

        # Publisher
        self.publisher = self.create_publisher(Float32, 'correction_voltage', 10)

        # Timer for control loop
        self.create_timer(self.dt, self.control_loop)

    def plank_callback(self, msg):
        if len(msg.data) >= 2:
            x_t_new = msg.data[0]
            y_t_new = msg.data[1]

            if self.coord_list_x:
                dist = np.sqrt((x_t_new - self.coord_list_x[-1])**2 + (y_t_new - self.coord_list_y[-1])**2)
                if dist > 0.20:
                    self.coord_list_x.append(x_t_new)
                    self.coord_list_y.append(y_t_new)
            else:
                self.coord_list_x.append(x_t_new)
                self.coord_list_y.append(y_t_new)

    def current_pose_callback(self, msg):
        if len(msg.data) >= 3:
            x_c = msg.data[0]
            y_c = msg.data[1]
            theta = msg.data[2]
            self.current_pose = (x_c, y_c, theta)

            if self.coord_list_x and self.coord_list_y:
                first_x = self.coord_list_x[0]
                first_y = self.coord_list_y[0]
                dist = np.sqrt((x_c - first_x)**2 + (y_c - first_y)**2)
                if dist <= 0.5:
                    self.coord_list_x.pop(0)
                    self.coord_list_y.pop(0)
                    self.get_logger().info(f"I removed a point")


            self.compute_angle_error()

    def compute_angle_error(self):
        if self.current_pose is None:
            self.angular_error = 0.0
            return

        if len(self.coord_list_x) == 0 or len(self.coord_list_y) == 0:
            self.angular_error = 0.0
            return

        # Current pose
        x_c, y_c, theta = self.current_pose

        # Oldest point in target list
        x_t = self.coord_list_x[0]
        y_t = self.coord_list_y[0]

        # Compute angle to target
        dx = x_t - x_c
        dy = y_t - y_c
        target_angle = np.arctan2(dy, dx)

        # Compute angle error
        angle_error = target_angle - theta

        # Normalize to [-π, π]
        angle_error = (angle_error + np.pi) % (2 * np.pi) - np.pi

        self.angular_error = angle_error

    def control_loop(self):
        if self.angular_error is None:
            return

        correction_voltage = self.pi_controller.compute(self.angular_error)

        msg = Float32()
        msg.data = correction_voltage
        self.publisher.publish(msg)

        #self.get_logger().info(f"Correction voltage: {correction_voltage:.2f} V, Angular error: {np.degrees(self.angular_error):.2f}°")

def main(args=None):
    rclpy.init(args=args)
    node = AngularErrorCorrectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

