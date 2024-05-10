import math
C1 = 2 * math.pi / 60
class DiffDriveKinematics():
    def __init__(self, wheel_radius, wheel_base_radius):
        self.wheel_radius_ = wheel_radius
        self.wheel_base_radius_ = wheel_base_radius
    
    def get_left_speed(self, linear_velocity, angular_velocity):
        return ((linear_velocity - (self.wheel_base_radius_ * angular_velocity)) * 1 * 60 / ((self.wheel_radius_) * 2 * math.pi))
    
    def get_right_speed(self, linear_velocity, angular_velocity):
        return ((linear_velocity + (self.wheel_base_radius_ * angular_velocity)) * 1 * 60 / ((self.wheel_radius_) * 2 * math.pi))

    def get_lin_vel(self, left_rpm, right_rpm):
        return ((left_rpm * C1 * self.wheel_radius_) + (right_rpm * C1 * self.wheel_radius_)) / 2.0
    
    def get_ang_vel(self, left_rpm, right_rpm):
        return ((right_rpm * 2 * math.pi * self.wheel_radius_ / 60) - (left_rpm * 2 * math.pi * self.wheel_radius_ / 60))/ (2.0 * self.wheel_base_radius_)