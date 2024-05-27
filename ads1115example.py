# sudo pip3 install adafruit-circuitpython-ads1x15

import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import time
 
# Initialize the I2C interface
i2c = busio.I2C(board.SCL, board.SDA)
 
# Create an  ADS1115 object
ads = ADS. ADS1115(i2c)
 
# Define the analog input channels
channel0 = AnalogIn(ads, ADS.P0)
channel1 = AnalogIn(ads, ADS.P1)
 
# Loop to read the analog inputs continuously
while True:
    print("Analog Value 0: ", channel0.value, "Voltage 0: ", channel0.voltage)
    print("Analog Value 1: ", channel1.value, "Voltage 1: ", channel1.voltage)
    
    # Delay for 1 second
    time.sleep(1)