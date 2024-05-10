import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from rclpy.clock import Clock
import time

from .ZltechDrive.ZltechDrive import *

class DriveModbus(Node):
    def __init__(self):
        super().__init__('drive_modbs')

        # Initialize Driver
        self.left_driver_ = ZltechDrive('/dev/ttyUSB0', 6)
        self.left_driver_.set_velocity(0)
        self.right_driver_ = ZltechDrive('/dev/ttyUSB0', 4)
        self.right_driver_.set_velocity(0)

        # cmd vel subscriber & callback
        self.twist_sub_ = self.create_subscription(Twist, 'cmd_vel', self.twist_callback, 10)
        self.target_vel_ = [0, 0]
        self.prev_twist_t_ = time.time()

    def twist_callback(self, msg):
        self.target_vel_ = [msg.linear.x, msg.angular.z]
        self.prev_twist_t_ = time.time()


def main(args=None):
    rclpy.init(args=args)
    drive_modbus = DriveModbus()
    rclpy.spin(drive_modbus)
    drive_modbus.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
