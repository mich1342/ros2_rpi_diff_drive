import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.clock import Clock

from .ZltechDrive.ZltechDrive import *

class DriveModbus(Node):
    def __init__(self):
        super().__init__('drive_modbs')
        self.left_driver_ = ZltechDrive('/dev/ttyUSB0', 6)
        self.left_driver_.set_velocity(10)
        self.get_logger().info(f"Speed: {self.left_driver_.get_velocity()}")

def main(args=None):
    rclpy.init(args=args)
    drive_modbus = DriveModbus()
    rclpy.spin(drive_modbus)
    drive_modbus.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
