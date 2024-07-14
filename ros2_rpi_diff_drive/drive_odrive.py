import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from rclpy.clock import Clock
import time

from .OdriveDrive.OdriveDrive import *
from .DiffDriveKinematics.DiffDriveKinematics import *

class DriveOdrive(Node):
    def __init__(self):
        super().__init__('drive_odrive')

        # Initialize Driver
        self.driver_ = OdriveDrive()
        self.driver_.set_velocity(0, 0)

        # Initiate Kinematis
        self.kinematics_ = DiffDriveKinematics(0.0899, 0.22)

        # cmd vel subscriber & callback
        self.twist_sub_ = self.create_subscription(Twist, 'cmd_vel', self.twist_callback, 10)
        self.target_vel_ = [0, 0]
        self.prev_twist_t_ = time.time()

        # timer to publish and send velocity command
        self.timer_period_ = 0.033
        self.timer_ = self.create_timer(self.timer_period_, self.timer_callback)

        # Odometry publisher
        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "base_footprint"
        self.odom_pub_ = self.create_publisher(Odometry, 'odom/unfiltered', 10)

        self.odom_x_pos_ = 0.0
        self.odom_y_pos_ = 0.0
        self.odom_heading_ = 0.0

        self.prev_t_ = time.time()


    def timer_callback(self):
        # Check timeout
        if(time.time() - self.prev_twist_t_ > 1):
            self.target_vel_ = [0,0]

        # Send Velocity to Motors
        left_rpm = self.kinematics_.get_left_speed(self.target_vel_[0], self.target_vel_[1])
        right_rpm = -self.kinematics_.get_right_speed(self.target_vel_[0], self.target_vel_[1])
        self.driver_.set_velocity(left_rpm, right_rpm)
        
        # Get Velocity data from Motors
        [left_rpm_act, right_rpm_act] = self.driver_.get_velocity()
        right_rpm_act *= -1
        ts = Clock().now()
        self.odom_msg_.header.stamp = ts.to_msg()

        angular_vel_z = self.kinematics_.get_ang_vel(left_rpm_act, right_rpm_act)
        linear_vel_x = self.kinematics_.get_lin_vel(left_rpm_act, right_rpm_act)
        linear_vel_y = 0

        # Get delta time from previous update
        vel_dt = time.time() - self.prev_t_

        delta_heading = angular_vel_z * vel_dt

        cos_h = math.cos(self.odom_heading_)
        sin_h = math.sin(self.odom_heading_)
        delta_x = (linear_vel_x * cos_h - linear_vel_y * sin_h) * vel_dt
        delta_y = (linear_vel_x * sin_h + linear_vel_y * cos_h) * vel_dt

        self.odom_x_pos_ += delta_x
        self.odom_y_pos_ += delta_y
        self.odom_heading_ += delta_heading
        
        q = self.euler_to_quat(0, 0, self.odom_heading_)

        self.odom_msg_.pose.pose.position.x = self.odom_x_pos_
        self.odom_msg_.pose.pose.position.y = self.odom_y_pos_
        self.odom_msg_.pose.pose.position.z = 0.0

        self.odom_msg_.pose.pose.orientation.x = q[1]
        self.odom_msg_.pose.pose.orientation.y = q[2]
        self.odom_msg_.pose.pose.orientation.z = q[3]
        self.odom_msg_.pose.pose.orientation.w = q[0]

        self.odom_msg_.pose.covariance[0] = 0.001
        self.odom_msg_.pose.covariance[7] = 0.001
        self.odom_msg_.pose.covariance[35] = 0.001

        self.odom_msg_.twist.twist.linear.x = linear_vel_x
        self.odom_msg_.twist.twist.linear.y = 0.0
        self.odom_msg_.twist.twist.linear.z = 0.0
        
        self.odom_msg_.twist.twist.angular.x = 0.0
        self.odom_msg_.twist.twist.angular.y = 0.0
        self.odom_msg_.twist.twist.angular.z = angular_vel_z

        self.odom_msg_.twist.covariance[0] = 0.0001
        self.odom_msg_.twist.covariance[7] = 0.0001
        self.odom_msg_.twist.covariance[35] = 0.0001

        self.odom_pub_.publish(self.odom_msg_)

        self.prev_t_ = time.time()


    def twist_callback(self, msg):
        self.target_vel_ = [msg.linear.x, msg.angular.z]
        self.prev_twist_t_ = time.time()

    def euler_to_quat(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0,0,0,0]
        
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = cy * cp * sr - sy * sp * cr
        q[2] = sy * cp * sr + cy * sp * cr
        q[3] = sy * cp * cr - cy * sp * sr

        return q


def main(args=None):
    rclpy.init(args=args)
    drive_odrive = DriveOdrive()
    rclpy.spin(drive_odrive)
    drive_odrive.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
