import gpiozero 
import time
import signal
i_start = gpiozero.Button(18)
i_selector = gpiozero.Button(23)
i_limit_switch = gpiozero.Button(24)
i_emergency = gpiozero.Button(16)
i_hard_reset = gpiozero.Button(20)
i_reset = gpiozero.Button(6)
i_input_1 = gpiozero.Button(13)
i_input_2 = gpiozero.Button(26)
i_input_3 = gpiozero.Button(21)

o_output_1 = gpiozero.LED(17)
o_output_2 = gpiozero.LED(27)
o_output_3 = gpiozero.LED(22)
o_output_4 = gpiozero.LED(5)

def print_msg():
    print("Test")
def main(args=None):
    i_start.when_pressed = print_msg
    signal.pause()
    
        
if __name__ == '__main__':
    main()