import time

from adafruit_extended_bus import ExtendedI2C as I2C
import adafruit_bno08x
from adafruit_bno08x.i2c import BNO08X_I2C

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

linear_accel = [0, 0, 0]  # x, y, z  (in m/s^2)
gyro = [0, 0, 0]  # x, y, z  (in deg/s)


class BNO085_Publisher(Node):
    def __init__(self):
        super().__init__("BNO085_Publisher")
        # create the publisher for the IMU data
        self.imu_data_publisher = self.create_publisher(
            Imu, "IMU_Data", 10  # ROS Message  # Topic
        )

        # IMU sensor (BNO085)
        self.imu = None
        self.init_sensor()

        self.read_send_timer = self.create_timer(0.05, self.read_and_send_imu_data)

    def init_sensor(self):
        i2c = I2C(3)
        try:
            self.imu = BNO08X_I2C(i2c, address=0x4B)
        except:
            self.get_logger().error("Failed to connect to BNO085 via I2C...")
            raise Exception("Failed to connect to BNO085 via I2")

        # enable the reports from the IMU

        self.imu.enable_feature(
            adafruit_bno08x.BNO_REPORT_ACCELEROMETER
        )  # Linear acceleration data
        self.imu.enable_feature(
            adafruit_bno08x.BNO_REPORT_GYROSCOPE
        )  # For Angular Velocity data

        time.sleep(0.5)  # Make sure we the IMU is initialized

    def read_and_send_imu_data(self):
        # get the Angular Velocity (gryo data) of the robot
        gyro[0], gyro[1], gyro[2] = self.imu.gyro
        # get the Linear Acceleration of the robot
        linear_accel[0], linear_accel[1], linear_accel[2] = self.imu.acceleration

        # create messages to publish
        imu_data_msg = Imu()

        imu_data_msg.angular_velocity.x = gyro[0]
        imu_data_msg.angular_velocity.y = gyro[1]
        imu_data_msg.angular_velocity.z = gyro[2]
        imu_data_msg.linear_acceleration.x = linear_accel[0]
        imu_data_msg.linear_acceleration.y = linear_accel[1]
        imu_data_msg.linear_acceleration.z = linear_accel[2]

        self.imu_data_publisher.publish(imu_data_msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        bno_publisher = BNO085_Publisher()
        rclpy.spin(bno_publisher)
        bno_publisher.destroy_node()
    except Exception as e:
        print(e)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == "__main__":
    main()
