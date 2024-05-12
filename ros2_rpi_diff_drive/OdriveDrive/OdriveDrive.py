import odrive 
from odrive.enums import *

class OdriveDrive():
    def __init__(self):
        self.odrv0_= odrive.find_any()
        self.odrv0_.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv0_.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        self.odrv0_.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv0_.axis1.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        
    def set_velocity(self, axis0, axis1):
        self.odrv0_.axis0.controller.input_vel= axis0 / 60
        self.odrv0_.axis1.controller.input_vel= axis1 / 60
    
    def get_velocity(self):
        return [self.odrv0_.axis0.encoder.vel_estimate, self.odrv0_.axis1.encoder.vel_estimate]

