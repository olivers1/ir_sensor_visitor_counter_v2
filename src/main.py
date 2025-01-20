import numpy as np
from enum import Enum
#from abc import ABC, abstractmethod
#from inspect import signature
import datetime

# Import SPI library (for hardware SPI) and MCP3008 library.
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008
import time
import RPi.GPIO as GPIO

# GPIO setup
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
ir_pin_led0 = 8     # IR-led for sensor0
GPIO.setup(ir_pin_led0,GPIO.OUT)
ir_pin_led1 = 10    # IR-led for sensor1		
GPIO.setup(ir_pin_led1,GPIO.OUT)

# pwm signal for IR-led pins
pi_pwm_led0 = GPIO.PWM (ir_pin_led0, 38000) # IR-led for sensor0
pi_pwm_led0.start(0)
pi_pwm_led0.ChangeDutyCycle(50)     # pwm signal duty cycle
pi_pwm_led1 = GPIO.PWM (ir_pin_led1, 38000) # IR-led for sensor1
pi_pwm_led1.start(0)
pi_pwm_led1.ChangeDutyCycle(50)     # pwm signal duty cycle

# Hardware SPI configuration:
SPI_PORT   = 0
SPI_DEVICE = 0

# ADC converter object (converts analog voltage to digital signal) from IR-sensor
mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))


class SensorTrigState(Enum):
    NO_TRIG = 0
    TRIG = 1
    UNKNOWN = 2


class SensorSample:
    def __init__(self):
        self.value: int = 0
        self.timestamp: int = 0
        self.trig_state = SensorTrigState.NO_TRIG
        
    def set(self, value, timestamp, trig_state):
        self.value = value
        self.timestamp = timestamp
        self.trig_state = trig_state
    
    def get(self):
        return self.value, self.timestamp, self.trig_state


class IrSensor:
    def __init__(self, mcp_channel :int, sensor_trig_threshold: int):
        self.mcp_channel = mcp_channel
        self.sensor_trig_threshold = sensor_trig_threshold
        
    def get_sensor_data(self):
        # read sensor value and timestamp
        value = mcp.read_adc(self.mcp_channel)
        timestamp = round(time.time()*1000)
        
        # evalute readout value to determine sensor trig
        trig_state = SensorTrigState.NO_TRIG
        if(value < self.sensor_trig_threshold):   # detect sensor trig. below threshold == trig, above threshold = no trig
            trig_state = SensorTrigState.TRIG    # trig detected
        else:
            trig_state = SensorTrigState.NO_TRIG  # no trig detected
        return value, timestamp, trig_state
    

class SensorHandler:
    pass
    
    # def __init__(self, number_of_sensors: int, max_samples: int):
    #     self.number_of_sensors = number_of_sensors
    #     self.max_samples = max_samples
    #     #self.sensor_logs = np.array([[SensorSample()]*max_samples] * self.number_of_sensors)    # create number of objects needed to store the buffered sensor data
    #     self.sensor_logs = np.array([[SensorSample() for _ in range(max_samples)] for _ in range(self.number_of_sensors)])    # create number of objects needed to store the buffered sensor data
        
    # def register_sample(self, sensor_id, index, value, timestamp, trig_state):
    #     self.sensor_logs[sensor_id][index].set(value, timestamp, trig_state)    
    
    # def get_sample(self, sensor_id: int, index: int):
    #     return self.sensor_logs[sensor_id][index]
    
    # def get_sensor_logs(self):
    #     return self.sensor_logs


def main():
    pass


if __name__ == "__main__":
   main()