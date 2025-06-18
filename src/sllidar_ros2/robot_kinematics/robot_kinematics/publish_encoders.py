import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import Int32MultiArray

arduino_port = '/dev/ttyUSB1'
baudrate = 9600

class EncoderPublisher(Node):
	def __init__(self):
		super().__init__('encoder_publisher')

		# Initialize serial connection
		self.serial_port = serial.Serial(arduino_port, baudrate, timeout=1)

		# ROS 2 publisher
		self.publisher_ = self.create_publisher(Int32MultiArray, 'encoder_counts', 10)

		# Timer to periodically check for data
		self.timer = self.create_timer(0.01, self.read_serial)

	def read_serial(self):
		try:
			line = self.serial_port.readline().decode().strip()
			if not line:
				return

			# Expecting format: X,Y
			parts = line.split(',')
			if len(parts) != 2:
				self.get_logger().warn(f"Unexpected serial format: {line}")
				return

			left = int(parts[0])
			right = int(parts[1])

			msg = Int32MultiArray()
			msg.data = [left, right]

			self.publisher_.publish(msg)
			self.get_logger().info(f"Published: {msg.data}")

		except Exception as e:
			self.get_logger().error(f"Error reading serial: {e}")

def main(args=None):
	rclpy.init(args=args)
	node = EncoderPublisher()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
