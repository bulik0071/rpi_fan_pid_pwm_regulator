import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import time
import serial
from pwm_avt_1825example import set_output_state,set_pwm,send_command,get_output_state

# Initialize the I2C interface
i2c = busio.I2C(board.SCL, board.SDA)
 
# Create an  ADS1115 object
ads = ADS. ADS1115(i2c)
 
# Define the analog input channels
potentiometer = AnalogIn(ads, ADS.P0)
mass_flow_sensor = AnalogIn(ads, ADS.P1)

ser = serial.Serial(
    port='/dev/ttyUSB0',
    baudrate=9600,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=1
)

#do all tasks below until q is pressed:
#read potentiometer - if more than 2% - then setup fan is on - set_output_state(module_address, 1), else set_output_state(module_address, 0)
#read mass flow sensor
#PID regulate to get desire mass flow value based on potentiometer reading if needed  - set_pwm(module_address, 128, 0)

module_address = 24

# Initialize PID controller variables
desired_flow = 0
Kp = 1.0
Ki = 0.1
Kd = 0.05
previous_error = 0
integral = 0

def pid_control(setpoint, measured_value):
    global previous_error, integral
    error = setpoint - measured_value
    integral += error
    derivative = error - previous_error
    output = Kp * error + Ki * integral + Kd * derivative
    previous_error = error
    return output

try:
    while True:
        pot_value = potentiometer.value / 65535  # Normalize potentiometer reading to range 0-1
        if pot_value > 0.02:
            set_output_state(module_address, 1)  # Turn on the fan
            desired_flow = pot_value * 100  # Scale desired flow value
        else:
            set_output_state(module_address, 0)  # Turn off the fan
            desired_flow = 0

        flow_value = mass_flow_sensor.value / 65535 * 100  # Normalize mass flow sensor reading to range 0-100

        if desired_flow > 0:
            control_signal = pid_control(desired_flow, flow_value)
            pwm_value = min(max(int(control_signal * 255 / 100), 0), 255)  # Scale control signal to 0-255 range
            set_pwm(module_address, pwm_value, 0)  # Set PWM value
        else:
            set_pwm(module_address, 0, 0)  # Turn off PWM

        print(f"Potentiometer: {pot_value:.2f}, Desired Flow: {desired_flow:.2f}, Measured Flow: {flow_value:.2f}, PWM: {pwm_value}")
        
        time.sleep(1)

except KeyboardInterrupt:
    print("Program stopped by user")
finally:
    ser.close()