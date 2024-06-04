import minimalmodbus
import serial

class ZltechDrive():
    def __init__(self, PORT, SLAVE_ID):
        # Initiate minimalmodbus
        self.drive_ =  minimalmodbus.Instrument(PORT, SLAVE_ID) 
        self.drive_.serial.baudrate = 115200         # Baud
        self.drive_.serial.bytesize = 8
        self.drive_.serial.parity   = serial.PARITY_NONE
        self.drive_.serial.stopbits = 1
        self.drive_.serial.timeout  = 0.05          # seconds
        self.drive_.mode = minimalmodbus.MODE_RTU   # rtu or ascii mode
        self.drive_.clear_buffers_before_each_transaction = True

        # Clear Alarm
        self.drive_.write_register(0x2031, 0x06, 0, 6, False)

        # Enable Driver
        self.drive_.write_register(0x2031, 0x08, 0, 6, False)

        # Velocity Mode
        self.drive_.write_register(0x2032, 0x08, 0, 6, False)

    def set_velocity(self, target):
        # Set 0 Velocity
        self.drive_.write_register(0x203A, int(target), 0, 6, True)
    
    def get_velocity(self):
        return self.drive_.read_register(0x202C, 1, 3, True)
    
    def get_voltage(self):
        return self.drive_.read_register(0x2029, 2, 3, False)
    
    def get_motor_temp(self):
        return self.drive_.read_register(0x2026, 1, 3, False)
    
    def get_driver_temp(self):
        return self.drive_.read_register(0x2024, 1, 3, False)

