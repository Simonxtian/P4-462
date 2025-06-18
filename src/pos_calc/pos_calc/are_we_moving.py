import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
import math

class AreWeMoving(Node):
	def __init__(self):
		super().__init__('are_we_moving')

		# Subscribe to linear topic
		self.subscription = self.create_subscription(
			Float32, 'linear_pot', self.linear_callback, 10)

		# Publisher for direction
		self.publisher = self.create_publisher(Float32MultiArray, 'forwards_backwards', 10)

		self.linear_val = 10

		self.timer = self.create_timer(0.05, self.calculate_and_publish)

	def linear_callback(self, msg):
		self.linear_val = msg.data  # Get linear potmeter value

	def calculate_and_publish(self):
		direction = Float32MultiArray()

		if 10 >= self.linear_val >= 8 :
			forward = 0.0
			backward = 0.0

		elif self.linear_val > 10:
			forward = 0.0
			backward = 1.0

		elif self.linear_val < 8:
			forward = 1.0
			backward = 0.0

		# Add the boolean values as bytes to the data field of the ByteMultiArray
		direction.data = [forward, backward]

		# Publish the result
		self.publisher.publish(direction)

		self.get_logger().info(f'Forwards {forward}, backward: {backward}')

def main(args=None):
	rclpy.init(args=args)
	node = AreWeMoving()
	rclpy.spin(node)

	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
