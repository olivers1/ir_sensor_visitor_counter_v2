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
        self.trig_state = SensorTrigState.UNKNOWN
        
    def set_sample(self, value, timestamp, trig_state):
        self.value = value
        self.timestamp = timestamp
        self.trig_state = trig_state
    
    def get_sample(self):
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
    def __init__(self, number_of_sensors: int, num_sample_columns: int):
        self.number_of_sensors = number_of_sensors
        self.num_sample_columns = num_sample_columns
        
        self.index_counter = 0
        self.sensor_log_sample_array = self.create_log_sample_array(self.number_of_sensors, self.num_sample_columns)
        
    def register_log_sample(self, sensor_id, value: int, timestamp: int, trig_state: SensorTrigState):
        # print(np.shape(self.sensor_log_sample_array))
        # print(self.sensor_log_sample_array)
        
        # check if there are any empty columns to store sample in, otherwise create more columns
        # check only when all sensors have stored their data samples, first sensor_id is zero '0'
        if(sensor_id == self.number_of_sensors - 1):
            #print(np.shape(self.sensor_log_sample_array)[1])
            if(np.shape(self.sensor_log_sample_array)[1] <= self.index_counter + 1):
                new_columns = self.create_log_sample_array(self.number_of_sensors, 1)   # create one extra column
                self.sensor_log_sample_array = np.append(self.sensor_log_sample_array, new_columns, 1)
            
        self.sensor_log_sample_array[sensor_id][self.index_counter].set_sample(value, timestamp, trig_state)

        # increase index_counter when all sensors have stored their data samples, first sensor_id is zero '0'
        if(sensor_id == self.number_of_sensors - 1):
            self.index_counter += 1
            return self.index_counter - 1   # value adjusted to return the index of last stored sample
        return self.index_counter

    def create_log_sample_array(self, number_of_sensors: int, num_of_columns):
        return np.array([[SensorSample()] * self.num_sample_columns] * self.number_of_sensors)
                                
    def get_log_sample(self, sensor_id, sample_index):
        return self.sensor_log_sample_array[sensor_id][sample_index]
    
    def get_sensor_log_sample_array(self):
        return self.sensor_log_sample_array


def main():
    sensor_trig_threshold = 800     # sensor digital value (0 - 1023) to represent IR-sensor detection, below threshold value == sensor trig
    number_of_sensors = 2
    sensors = []
    initial_num_sample_columns = 1

    for sensor_id in range(number_of_sensors):
        sensors.append(IrSensor(sensor_id, sensor_trig_threshold))
    
    sensor_handler = SensorHandler(number_of_sensors, initial_num_sample_columns)

    i = 0
    while(i < 10):
        for sensor_id, sensor in enumerate(sensors):
            index_counter = sensor_handler.register_log_sample(sensor_id, *sensor.get_sensor_data())    # '*' unpacks the return tuple from function call)
            print(f"({int(sensor_id)}, {index_counter})")
            print(sensor_handler.sensor_log_sample_array[sensor_id][index_counter].value, sensor_handler.sensor_log_sample_array[sensor_id][index_counter].timestamp, sensor_handler.sensor_log_sample_array[sensor_id][index_counter].trig_state.name)
            
        i += 1
    
    #print(sensor_handler.get_sensor_log_sample_array())



if __name__ == "__main__":
   main()