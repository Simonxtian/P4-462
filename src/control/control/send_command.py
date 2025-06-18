import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, Int32MultiArray
import serial

class MotorSerialSender(Node):
    def __init__(self):
        super().__init__('motor_serial_sender')

        # Initialize serial
        self.ser = serial.Serial('/dev/ttyESP32', 115200, timeout=1)

        # Latest values
        self.left_voltage = 0.0
        self.right_voltage = 0.0
        self.correction = 0.0
        self.forward = 0

        # Subscriptions
        self.create_subscription(
            Float32MultiArray,
            'motor_voltage_straight',
            self.voltage_callback,
            10
        )

        self.create_subscription(
            Float32,
            'correction_voltage',
            self.correction_callback,
            10
        )

        self.create_subscription(
            Float32MultiArray,
            'forwards_backwards',
            self.forward_callback,
            10
        )

        # Timer for sending commands
        self.create_timer(0.05, self.send_serial_commands)  # every 100ms

    def voltage_callback(self, msg):
        self.left_voltage = msg.data[0]
        self.right_voltage = msg.data[1]

    def correction_callback(self, msg):
        self.correction = msg.data
        #self.correction = 0

    def forward_callback(self, msg):
        self.forward = msg.data[0]

    def send_serial_commands(self):
        left_corrected = (self.left_voltage + self.correction)*self.forward
        right_corrected = (self.right_voltage - self.correction)*self.forward

        # Send left command
        cmd_left = f"1 {left_corrected:.3f}\n"
        self.ser.write(cmd_left.encode())
        response_left = self.ser.readline().decode().strip()
        self.get_logger().info(f"Sent: {cmd_left.strip()} | Arduino replied: {response_left}")

        # Send right command
        cmd_right = f"2 {right_corrected:.3f}\n"
        self.ser.write(cmd_right.encode())
        response_right = self.ser.readline().decode().strip()
        self.get_logger().info(f"Sent: {cmd_right.strip()} | Arduino replied: {response_right}")

def main(args=None):
    rclpy.init(args=args)
    node = MotorSerialSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
