import rclpy

from rclpy.node import Node

from sensor_msgs.msg import Imu

import board

import busio

import adafruit_bno055

 

class BNO055Node(Node):

    def __init__(self):

        super().__init__('imu')

        self.publisher_ = self.create_publisher(Imu, 'imu/data_raw', 10)

        self.timer = self.create_timer(1.0, self.timer_callback) 

      

        self.i2c = busio.I2C(board.SCL, board.SDA)

        self.bno = adafruit_bno055.BNO055_I2C(self.i2c)
        
        self.calibrated = False
        
        self.check_calibration()
 

    def timer_callback(self):

        if not self.calibrated:

            self.check_calibration()

        if self.calibrated:

            imu_msg = Imu()

            imu_msg.header.stamp = self.get_clock().now().to_msg()

            imu_msg.header.frame_id = 'imu_frame'

 

            accel = self.bno.acceleration

            gyro = self.bno.gyro

            magnetic = self.bno.magnetic

            euler = self.bno.euler

            quaternion = self.bno.quaternion

            temperature = self.bno.temperature

 

            imu_msg.linear_acceleration.x = accel[0] if accel else 0.0

            imu_msg.linear_acceleration.y = accel[1] if accel else 0.0

            imu_msg.linear_acceleration.z = accel[2] if accel else 0.0

 

            imu_msg.angular_velocity.x = gyro[0] if gyro else 0.0

            imu_msg.angular_velocity.y = gyro[1] if gyro else 0.0

            imu_msg.angular_velocity.z = gyro[2] if gyro else 0.0

 

            imu_msg.orientation.x = quaternion[0] if quaternion else 0.0

            imu_msg.orientation.y = quaternion[1] if quaternion else 0.0

            imu_msg.orientation.z = quaternion[2] if quaternion else 0.0

            imu_msg.orientation.w = quaternion[3] if quaternion else 0.0

 

            imu_msg.orientation_covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 

 

            self.publisher_.publish(imu_msg)

    def check_calibration(self):
        
        if self.bno.calibration_status[0] == 3 and self.bno.calibration_status[1] == 3:
        
            self.calibrated = True
            
            self.get_logger().info('IMU is calibrated.')
        
        else:
            
            self.get_logger().warn('IMU is not yet calibrated. Waiting for calibration...')

     

def main(args=None):

    rclpy.init(args=args)

    node = BNO055Node()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

 

if __name__ == '__main__':

    main()
