import RPi.GPIO as GPIO
import time

import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn

GPIO.setmode(GPIO.BCM)



pwm_control_pin=26
GPIO.setwarnings(False)

#create the spi bus
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)
# create the cs (chip select)
cs = digitalio.DigitalInOut(board.D22)
# create the mcp object
mcp = MCP.MCP3008(spi, cs)
# create an analog input channel on pin 0
chan0 = AnalogIn(mcp, MCP.P0)
chan7 = AnalogIn(mcp, MCP.P7)

print('Raw ADC Value: ', chan0.value)
print('ADC Voltage: ' + str(chan0.voltage) + 'V')
last_read = 0       # this keeps track of the last potentiometer value
tolerance = 500     # to keep from being jittery we'll only change
                    # volume when the pot has moved a significant amount
                    # on a 16-bit ADC
                    
def remap_range(value, left_min, left_max, right_min, right_max):
    # this remaps a value from original (left) range to new (right) range
    # Figure out how 'wide' each range is
    left_span = left_max - left_min
    right_span = right_max - right_min

    # Convert the left range into a 0-1 range (int)
    valueScaled = int(value - left_min) / int(left_span)

    # Convert the 0-1 range into a value in the right range.
    return int(right_min + (valueScaled * right_span))

def valve_control (current_flow):
    pwm_hz=15
    pwm_load=50.000
    Flow = float(remap_range(chan0.value, 59, 65535, 0, 10000))
    if Flow<0:
        Flow=0
    print('Flow: ',Flow)
    current_flow=Flow
    target_flow=2000
    diff_flow=target_flow/(current_flow+0.01)
    GPIO.setup(pwm_control_pin, GPIO.OUT)
    pwm_valve=GPIO.PWM(pwm_control_pin,pwm_hz)
    pwm_valve.start(pwm_load)
    while (True):
        Flow = float(remap_range(chan0.value, 59, 65535, 0, 10000))
        if Flow<0:
            Flow=0
        current_flow=Flow
        if current_flow==0:
            current_flow=0.01
        diff_flow=target_flow/(current_flow)
        pwm_load=pwm_load*diff_flow
        if (pwm_load>100):
            pwm_load=100
        elif (pwm_load<0):
            pwm_load=0
        elif (pwm_load==0):
            pwm_load=0.0001
        pwm_valve.ChangeDutyCycle(pwm_load)
        
        time.sleep(1)
        print('Flow: ',Flow)
        print('PWM: ',pwm_load)

        
valve_control (1000)
#valve_control (float(app.remap_range(0, 0, 65535, 0, 10000)))

GPIO.cleanup()
print('done')
