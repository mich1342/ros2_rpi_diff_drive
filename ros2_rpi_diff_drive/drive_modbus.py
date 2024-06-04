import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Float32

from rclpy.clock import Clock
import time

from .ZltechDrive.ZltechDrive import *
from .DiffDriveKinematics.DiffDriveKinematics import *

import gpiozero 

class DriveModbus(Node):
    def __init__(self):
        super().__init__('drive_modbs')

        # Initialize Driver
        self.left_driver_ = ZltechDrive('/dev/ttyUSB0', 6)
        self.left_driver_.set_velocity(0)
        self.right_driver_ = ZltechDrive('/dev/ttyUSB0', 4)
        self.right_driver_.set_velocity(0)

        # Initiate Kinematis
        self.kinematics_ = DiffDriveKinematics(0.099, 0.22)

        # cmd vel subscriber & callback
        self.twist_sub_ = self.create_subscription(Twist, 'cmd_vel', self.twist_callback, 10)
        self.target_vel_ = [0, 0]
        self.prev_twist_t_ = time.time()

        # timer to publish and send velocity command
        self.timer_ = self.create_timer(0.33, self.timer_callback)

        # timer to publish statistics data
        self.timer_ = self.create_timer(5, self.statistics_callback)
        self.voltage_pub_ = self.create_publisher(Float32, 'talosbot1/bus_voltage', 10)
        self.left_motor_temp_pub_ = self.create_publisher(Float32, 'talosbot1/temperature/left_motor_temp', 10)
        self.left_driver_temp_pub_ = self.create_publisher(Float32, 'talosbot1/temperature/left_driver_temp', 10)
        self.right_motor_temp_pub_ = self.create_publisher(Float32, 'talosbot1/temperature/right_motor_temp', 10)
        self.right_driver_temp_pub_ = self.create_publisher(Float32, 'talosbot1/temperature/right_driver_temp', 10)


        self.timer_ = self.create_timer(0.5, self.light_callback)
        self.light_publisher = self.create_publisher(Int32, 'talosbot1/light_status', 10)
        # Odometry publisher
        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "base_footprint"
        self.odom_pub_ = self.create_publisher(Odometry, 'odom/unfiltered', 10)

        self.odom_x_pos_ = 0.0
        self.odom_y_pos_ = 0.0
        self.odom_heading_ = 0.0

        self.emergency_state = False
        self.i_start = gpiozero.Button(18)
        self.i_selector = gpiozero.Button(23)
        self.i_limit_switch = gpiozero.Button(24)
        self.i_emergency = gpiozero.Button(16)
        self.i_hard_reset = gpiozero.Button(20)
        self.i_reset = gpiozero.Button(6)
        self.i_input_1 = gpiozero.Button(13)
        self.i_input_2 = gpiozero.Button(26)
        self.i_input_3 = gpiozero.Button(21)

        self.o_output_1 = gpiozero.LED(27)
        self.o_output_2 = gpiozero.LED(5)
        self.o_output_3 = gpiozero.LED(17)
        self.o_output_4 = gpiozero.LED(22)   

        self.i_start.when_pressed = self.release_emergency
        
        self.i_emergency.when_released = self.set_emergency
        self.i_limit_switch.when_pressed = self.set_emergency
        
        if not(self.i_emergency.is_pressed) or self.i_limit_switch.is_pressed:
            self.set_emergency()

        self.prev_t_ = time.time()

    def statistics_callback(self):
        msg_ = Float32()
        
        # Bus Voltage Publisher
        left_voltage = float(self.left_driver_.get_voltage())
        right_voltage = float(self.right_driver_.get_voltage())
        msg_.data = (left_voltage + right_voltage) / 2
        self.voltage_pub_.publish(msg_)

        msg_.data = float(self.left_driver_.get_driver_temp())
        self.left_motor_temp_pub_.publish(msg_)        
        msg_.data = float(self.left_driver_.get_motor_temp())
        self.left_driver_temp_pub_.publish(msg_)
        msg_.data = float(self.right_driver_.get_driver_temp())
        self.right_motor_temp_pub_.publish(msg_)
        msg_.data = float(self.right_driver_.get_motor_temp())
        self.right_driver_temp_pub_.publish(msg_)
        


    def timer_callback(self):
        # Check timeout
        if(time.time() - self.prev_twist_t_ > 0.5):
            self.target_vel_ = [0,0]

        # Calculate Velocity to Motors
        left_rpm = self.kinematics_.get_left_speed(self.target_vel_[0], self.target_vel_[1])
        right_rpm = self.kinematics_.get_right_speed(self.target_vel_[0], self.target_vel_[1])
        
        # Check Emergency State
        if self.emergency_state or not(self.i_emergency.is_pressed) or self.i_limit_switch.is_pressed:
            left_rpm = 0
            right_rpm = 0

        self.left_driver_.set_velocity(-left_rpm)
        self.right_driver_.set_velocity(right_rpm)

        # Get Velocity data from Motors
        left_rpm_act = -self.left_driver_.get_velocity()
        right_rpm_act = self.right_driver_.get_velocity()

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

    def light_callback(self):
        linear = self.target_vel_[0]
        angular = self.target_vel_[1]

        LINEAR_THRESHOLD = 0.02
        ANGULAR_THRESHOLD = 0.01
        msg = Int32()

        if abs(linear) < LINEAR_THRESHOLD and abs(angular) < ANGULAR_THRESHOLD:
            msg.data = 10
        else:
            if abs(angular) < ANGULAR_THRESHOLD:
                msg.data = 20
            else:
                if angular > 0:
                    msg.data = 30
                else:
                    msg.data = 40
        if self.emergency_state:
            msg.data = 0
        
        if msg.data == 0:
            self.o_output_1.on()
            self.o_output_2.off()
            self.o_output_3.off()
            self.o_output_4.off()
        if msg.data > 10:
            self.o_output_1.off()
            self.o_output_2.off()
            self.o_output_3.on()
            self.o_output_4.off()
        if msg.data == 10:
            self.o_output_1.off()
            self.o_output_2.off()
            self.o_output_3.off()
            self.o_output_4.off()
        

        self.light_publisher.publish(msg)
        

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
    def set_emergency(self):
        self.emergency_state = True
        print("emergency")

    def release_emergency(self):
        if not(self.i_emergency.is_pressed) or self.i_limit_switch.is_pressed:
            self.set_emergency()
        else:
            self.emergency_state = False
            print("not emergency")

def main(args=None):
    rclpy.init(args=args)
    drive_modbus = DriveModbus()
    rclpy.spin(drive_modbus)
    drive_modbus.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
