
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray
import numpy as np

class PIController:
    def __init__(self, kp, ki, dt, output_limits=(-48.0, 48.0)):
        self.kp = kp
        self.ki = ki
        self.dt = dt
        self.integral = 0
        self.output_limits = output_limits

    def reset(self):
        self.integral = 0

    def compute(self, target_speed, actual_speed):
        error = target_speed - actual_speed
        self.integral += error * self.dt
        output = self.kp * error + self.ki * self.integral
        min_out, max_out = self.output_limits
        return max(min_out, min(max_out, output))
        #return output

class PIControllerNode(Node):
    def __init__(self):
        super().__init__('pi_controller_node')

        self.left_pi = PIController(kp=4.0, ki=30.8, dt=0.1, output_limits=(-10.0, 10.0))
        self.right_pi = PIController(kp=4.0, ki=30.8, dt=0.1, output_limits=(-10.0, 10.0))

        self.total_pi = PIController(kp=4.0, ki=30.8, dt=0.05, output_limits=(-10.0, 10.0))

        self.subscription = self.create_subscription(
            Float32MultiArray,
            'encoder',
            self.encoder_callback,
            10
        )

        self.subscription_lin = self.create_subscription(
            Float32MultiArray,
            'forwards_backwards',
            self.lin_callback,
            10
        )

        self.publisher = self.create_publisher(
            Float32MultiArray,
            'motor_voltage_straight',
            10
        )

        self.timer = self.create_timer(0.05, self.control_loop)
        self.last_left_rev = 0
        self.last_right_rev = 0
        self.current_left_rev = 0
        self.current_right_rev = 0
        self.lin = 0.0

        self.target_speed_left = 0.5  # m/s
        self.target_speed_right = 0.5  # m/s

        self.target_speed_total = 1.1

    def encoder_callback(self, msg):
        self.current_left_rev = msg.data[0]
        self.current_right_rev = msg.data[1]

    def lin_callback(self, msg):
        self.lin = msg.data[0]

    def control_loop(self):
        if self.current_left_rev is None or self.last_left_rev is None:
            self.last_left_rev = self.current_left_rev
            self.last_right_rev = self.current_right_rev
            return

        dt = 0.05  # 100 ms

        speed_left = (self.current_left_rev - self.last_left_rev) / dt
        speed_right = (self.current_right_rev - self.last_right_rev) / dt

        speed_total = (speed_left + speed_right)*0.5*np.pi*2*0.1
        # Optional: Convert to m/s if needed by multiplying with wheel circumference
        actual_speed_left = speed_left
        actual_speed_right = speed_right

        actual_speed_total = speed_total

        total2_target_speed = self.target_speed_total * self.lin

        voltage_left = self.left_pi.compute(total2_target_speed, actual_speed_left)
        voltage_right = self.right_pi.compute(total2_target_speed, actual_speed_right)

        total_target_speed = self.target_speed_total * self.lin
        voltage_total = self.total_pi.compute(total_target_speed, actual_speed_total)

        # Publish voltages
        msg = Float32MultiArray()
        msg.data = [voltage_total, voltage_total]
        self.publisher.publish(msg)

        # Log
        self.get_logger().info(f"Published voltages - Left: {voltage_left:.2f} V, Right: {voltage_right:.2f} V, total: {voltage_total:.2f} V")

        self.last_left_rev = self.current_left_rev
        self.last_right_rev = self.current_right_rev

def main(args=None):
    rclpy.init(args=args)
    node = PIControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
