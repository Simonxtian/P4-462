import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import struct

#Adjust based on system
arduino_port = '/dev/ttyAUNO'
baudrate = 9600

def compute_checksum(lin_value, angle_rot, angle_pitch):
    data = f"{lin_value:.2f},{angle_rot:.2f},{angle_pitch:.2f}"
    checksum = 0

    for char in data:
        checksum ^= ord(char)

    return checksum

class ArduinoDataPublisher(Node):
    def __init__(self):
        super().__init__('arduino_data_publisher')

        # Initialize publishers
        self.linear_pub = self.create_publisher(Float32, 'linear_pot', 10)
        self.rotation_pub = self.create_publisher(Float32, 'rotation_pot', 10)
        self.pitch_pub = self.create_publisher(Float32, 'pitch_pot', 10)
        
        # Open serial connection
        self.serial_port = serial.Serial(arduino_port, baudrate, timeout=1)
        
        # Timer to read serial data periodically
        self.timer = self.create_timer(0.05, self.read_serial_data)
    
    def read_serial_data(self):
        try:
            line = self.serial_port.readline().decode().strip()
            if not line:
                return
            
            data = list(map(float, line.split(',')))
            if len(data) != 4:
                self.get_logger().warn(f'Invalid data format: {line}')
                return
            
            linear, rotation, pitch, received_checksum = data
            computed_checksum = compute_checksum(linear, rotation, pitch)
            
            if int(received_checksum) != computed_checksum:
                self.get_logger().warn(f'Checksum mismatch: {line}')
                return
            
            #Publish data to ROS 2 topics
            self.publish_data(linear, rotation, pitch)
        except Exception as e:
            self.get_logger().error(f'Error reading serial data: {e}')
    
    def publish_data(self, linear, rotation, pitch):
        self.linear_pub.publish(Float32(data=linear))
        self.rotation_pub.publish(Float32(data=rotation))
        self.pitch_pub.publish(Float32(data=pitch))
        self.get_logger().info(f'Published: Linear={linear}, Rotation={rotation}, Pitch={pitch}')

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoDataPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
