import rclpy
from rclpy.node import Node
import board
import busio
import adafruit_bno055

class BNO055Node(Node):

    def __init__(self):
        super().__init__('imu')
        # Timer to call the callback function every second
        self.timer = self.create_timer(1.0, self.timer_callback)
        # Initialize I2C and BNO055 sensor
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.bno = adafruit_bno055.BNO055_I2C(self.i2c)

    def timer_callback(self):
        # Read sensor data
        euler = self.bno.euler

        # Print Euler angles to the terminal
        if euler:
            roll, pitch, yaw = euler
            self.get_logger().info(f"Euler Angles - Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}")
        else:
            self.get_logger().info("Euler Angles data not available")

def main(args=None):
    rclpy.init(args=args)
    node = BNO055Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
