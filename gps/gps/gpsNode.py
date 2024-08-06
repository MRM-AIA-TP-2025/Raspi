import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import serial
import pynmea2


class GPSReaderNode(Node):
    def __init__(self):
        super().__init__('gps_reader_node')

        self.declare_parameter('serial_port', '/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('topic_name', 'gps/fix')

        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value

        try:
            self.ser = serial.Serial(serial_port, baud_rate, timeout=1)
            self.get_logger().info(f"Connected to GPS on {serial_port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to GPS on {serial_port}: {e}")
            return

        self.gps_publisher = self.create_publisher(NavSatFix, topic_name, 10)

        self.timer = self.create_timer(0.01, self.read_gps_data)

    def read_gps_data(self):
        try:
            line = self.ser.readline().decode('utf-8').strip()

            if line.startswith('$GNGGA') or line.startswith('$GPGGA'):
                msg = pynmea2.parse(line)

                gps_msg = NavSatFix()
                gps_msg.header.frame_id = 'gps'
                gps_msg.header.stamp = self.get_clock().now().to_msg()

                gps_msg.latitude = msg.latitude
                gps_msg.longitude = msg.longitude
                gps_msg.altitude = msg.altitude

                gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
                gps_msg.status.service = 1

                self.get_logger().info(f"Publishing GPS data: Lat {msg.latitude}, Lon {msg.longitude}, Alt {msg.altitude}")
                self.gps_publisher.publish(gps_msg)

        except pynmea2.nmea.ParseError as e:
            self.get_logger().error(f"Parse error: {e}")
        except UnicodeDecodeError as e:
            self.get_logger().error(f"Unicode decode error: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")

def main(args=None):
    rclpy.init(args=args)
    gps_reader_node = GPSReaderNode()
    rclpy.spin(gps_reader_node)
    gps_reader_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

