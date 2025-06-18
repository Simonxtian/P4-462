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

        self.dt = 0.05  # Control loop time step
        self.pi_controller = PIController(kp=3.98, ki=0.0, dt=self.dt, output_limits=(-10, 10))

        self.angular_error = 0.0
        self.current_pose = (0.0, 0.0, 0.0)  # (x, y, theta)
        self.target_points = []   # List of (x, y)
        self.reached_threshold = 1.0  # Distance threshold to consider a point "reached"
        self.min_distance_between_points = 0.1  # 10 cm
        self.state = 0
        self.average = None
        # Subscriptions
        self.create_subscription(Float32MultiArray, 'robot_pose', self.current_pose_callback, 10)
        self.create_subscription(Float32MultiArray, 'plank_tracking', self.plank_callback, 10)

        # Publisher
        self.publisher = self.create_publisher(Float32, 'correction_voltage', 10)

        # Timer for control loop
        self.create_timer(self.dt, self.control_loop)

    def plank_callback(self, msg):
        if len(msg.data) >= 2:
            #rotMatrix = np.array([
            #    [np.cos(self.current_pose[2]),  np.sin(self.current_pose[2])],
            #    [-np.sin(self.current_pose[2]), np.cos(self.current_pose[2])]
            #])
            #x_t, y_t = np.dot(rotMatrix, [msg.data[0],msg.data[1]])
            #x_t += self.current_pose[0]
            #y_t += self.current_pose[1]
            #new_target = (x_t, y_t)

            x_t = msg.data[0]
            y_t = msg.data[1]
            new_target = (x_t, y_t)

            if not self.target_points:
                self.target_points.append(new_target)
                #self.get_logger().info(f"1 added {new_target}")
            else:
                last_target = self.target_points[-1]
                dist = np.linalg.norm(np.array(new_target) - np.array(last_target))
                if dist >= self.min_distance_between_points:
                    self.target_points.append(new_target)
                    #self.get_logger().info(f"2 added {new_target}")

    def current_pose_callback(self, msg):
        if len(msg.data) >= 3:
            x_c = msg.data[0]
            y_c = msg.data[1]
            theta = msg.data[2]
            self.current_pose = (x_c, y_c, theta)
            self.compute_angle_error()

    def compute_angle_error(self):
        if self.current_pose is None or not self.target_points:
            self.angular_error = 0.0
            return

        x_c, y_c, theta = self.current_pose
        if self.average is not None:
            x_t, y_t = self.average
        else:
            x_t, y_t = self.target_points[0]

        dx = x_t - x_c
        dy = y_t - y_c
        target_angle = np.arctan2(dy, dx)

        angle_error = target_angle - theta
        angle_error = (angle_error + np.pi) % (2 * np.pi) - np.pi

        self.angular_error = angle_error

    def control_loop(self):
        if self.angular_error is None or self.current_pose is None or not self.target_points:
            return

        # Check if robot is close enough to the first point
        x_c, y_c, theta = self.current_pose

        # Moving average
        if len(self.target_points) < 5 and self.state == 0:
            sum_x = 0
            sum_y = 0
            for i in range(len(self.target_points)):
                x_t, y_t = self.target_points[i]
                sum_x += x_t
                sum_y += y_t
            self.average = (sum_x/len(self.target_points), sum_y/len(self.target_points))
        elif len(self.target_points) >= 5 and self.state == 0:
            sum_x = 0
            sum_y = 0
            for i in range(5):
                x_t, y_t = self.target_points[i]
                sum_x += x_t
                sum_y += y_t
            self.average = (sum_x/5, sum_y/5)
            self.state = 1

        x_t, y_t = self.target_points[0]
        self.get_logger().info(f"Target ({self.average[0]:.2f}, {self.average[1]:.2f}), Pose ({x_c:.2f}, {y_c:.2f}, {theta:.2f}), Error {self.angular_error}, Targets: {len(self.target_points)}")

        #dist_to_target = np.linalg.norm(np.array([x_t - x_c, y_t - y_c]))

        #if dist_to_target < self.reached_threshold:
        #    self.get_logger().info(f"Reached target ({x_t:.2f}, {y_t:.2f}), removing from list.")
        #    self.target_points.pop(0)

        x_t_trans = x_t - x_c
        y_t_trans = y_t - y_c
        rot_matrix = np.array([
            [np.cos(-theta), -np.sin(-theta)],
            [np.sin(-theta), np.cos(-theta)]
        ])
        x_t_local, y_t_local = np.dot(rot_matrix, [x_t_trans, y_t_trans])

        if x_t_local < 0.6 and -1 < y_t_local < 1:
            self.state = 0
            #self.get_logger().info(f"Reached target ({x_t:.2f}, {y_t:.2f}), removing from list.")
            self.target_points = self.target_points[1:]


        # Compute correction voltage
        correction_voltage = self.pi_controller.compute(self.angular_error)

        # Publish result
        msg = Float32()
        msg.data = correction_voltage * 0.4
        self.publisher.publish(msg)

        # Debugging info
        #self.get_logger().info(
        #    f"Voltage: {correction_voltage:.2f} V | Error: {np.degrees(self.angular_error):.2f}° | Targets: {len(self.target_points)}"
        #)


def main(args=None):
    rclpy.init(args=args)
    node = AngularErrorCorrectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
