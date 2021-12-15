from flask import Flask,render_template,url_for,request,redirect, make_response
import random
import json
import os
from time import time
from time import sleep
from random import random
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
import RPi.GPIO as GPIO


app = Flask(__name__)


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

def getCPUtemperature():
    res = os.popen('vcgencmd measure_temp').readline()
    temp =(res.replace("temp=","").replace("'C\n",""))
    #print("temp is {0}".format(temp)) #Uncomment here for testing
    return temp

def remap_range(value, left_min, left_max, right_min, right_max):
    # this remaps a value from original (left) range to new (right) range
    # Figure out how 'wide' each range is
    left_span = left_max - left_min
    right_span = right_max - right_min

    # Convert the left range into a 0-1 range (int)
    valueScaled = int(value - left_min) / int(left_span)

    # Convert the 0-1 range into a value in the right range.
    return int(right_min + (valueScaled * right_span))

def tflow(tf):
    print (tflow.tf)
    return tflow.tf

@app.route('/')
def main():
    return render_template('index.html')

# Here in this function we receive the target temperature from user
@app.route('/', methods=["POST"])
def formm():
    stf=request.form['tar']
    tflow.tf=int(stf)
    print ('ok')
    print (tflow.tf)
    
   
    return render_template('index.html', targetflow=stf)

# Here we send the sensors results to the HTML
@app.route('/data', methods=["GET", "POST"])
def data():
    # Data Format
    # [TIME, Temperature, Flow]

    Temperature = float(getCPUtemperature())
    Flow = float(remap_range(chan0.value, 59, 65535, 0, 10000))
    if Flow<0:
        Flow=0
    Vacuum = float(remap_range(chan7.value, 0, 65535, 0, 2000))
    
    data = [time() * 1000, Temperature, Flow, Vacuum]

    response = make_response(json.dumps(data))

    response.content_type = 'application/json'




    return response

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=8080, debug=True)
    #from waitress import serve
    #serve(app, host="0.0.0.0", port=8080)

