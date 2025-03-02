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

    def __eq__(self, other):    # used by np.all() to verify if trig_state for all elements in each row have the same value
        if isinstance(other, SensorSample):
            return self.trig_state == other.trig_state
        return False


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
    def __init__(self, number_of_sensors: int, num_sample_columns: int, num_consecutive_trigs):
        self.number_of_sensors = number_of_sensors
        self.num_sample_columns = num_sample_columns
        self.num_consecutive_trigs = num_consecutive_trigs
        
        self.index_counter = 0
        self.sensor_log_sample_array = self.create_log_sample_array(self.number_of_sensors, self.num_sample_columns)
        self.consecutive_num_trigs_array = self.create_log_sample_array(self.number_of_sensors, self.num_consecutive_trigs)
        
    def register_log_sample(self, sensor_id, value: int, timestamp: int, trig_state: SensorTrigState):
        # check if there are any empty columns to store sample in, otherwise create more columns
        # check only when all sensors have stored their data samples, first sensor_id is zero '0'
        if(sensor_id == self.number_of_sensors - 1):
            #print(np.shape(self.sensor_log_sample_array)[1])
            if(np.shape(self.sensor_log_sample_array)[1] <= self.index_counter + 1):    # check if all array columns are occupied
                new_columns = self.create_log_sample_array(self.number_of_sensors, 1)   # create an extra column
                self.sensor_log_sample_array = np.append(self.sensor_log_sample_array, new_columns, 1)
            
        self.sensor_log_sample_array[sensor_id][self.index_counter].set_sample(value, timestamp, trig_state)

        # increase index_counter when all sensors have stored their data samples, first sensor_id is zero '0'
        if(sensor_id == self.number_of_sensors - 1):
            self.index_counter += 1
            return self.index_counter - 1   # value adjusted to return the index of last stored sample
        return self.index_counter

    def create_log_sample_array(self, number_of_sensors: int, num_of_columns):
        return np.array([[SensorSample() for _ in range(num_of_columns)] for _ in range(number_of_sensors)], dtype=object)    # create an number_of_sensors dimensional array
                                
    def get_log_sample(self, sensor_id, sample_index):
        return self.sensor_log_sample_array[sensor_id][sample_index]
    
    def get_sensor_log_sample_array(self):
        return self.sensor_log_sample_array


class AppLoggingState(Enum):
    INIT = 0
    IDLE = 1
    LOGGING = 2
    RESULT_EVALUATION = 3


class IdentifiedMotionDirection(Enum):
    UNKNOWN = 0
    EXIT = 1
    ENTRY = 2


class TrigEvaluationManager:
    def __init__(self):
        self.sensor_trig_threshold = 800     # sensor digital value (0 - 1023) to represent IR-sensor detection, below threshold value == sensor trig
        self.number_of_sensors = 2
        self.sensors = []   # list containing all sensors
        self.initial_num_sample_columns = 1     # specifies number of columns for the initial log array
        self.readout_frequency = 0.5     # Hz
        self.index_counter = 0      # current index of sensor_log_sample_array
        self.num_consecutive_trigs = 3      # number of sensor trigs in a consecutive order to count it as a trig
        self.current_state = AppLoggingState.INIT
        self.index_log_start = 0
        self.index_log_stop = 0

        self.app_logging_state = AppLoggingState.INIT   # initial app logging state
        self.sensor_handler = SensorHandler(self.number_of_sensors, self.initial_num_sample_columns, self.num_consecutive_trigs)

    def run(self):
        for sensor_id in range(self.number_of_sensors):
            self.sensors.append(IrSensor(sensor_id, self.sensor_trig_threshold))
    
        while(True):    # add while(self.app_logging_state == AppLoggingState.LOGGING)
            for sensor_id, sensor in enumerate(self.sensors):
                self.index_counter = self.sensor_handler.register_log_sample(sensor_id, *sensor.get_sensor_data())    # '*' unpacks the return tuple from function call)
                print(f"(sensor_id, index_counter: {int(sensor_id)}, {self.index_counter})")
                print(self.sensor_handler.sensor_log_sample_array[sensor_id][self.index_counter].value, self.sensor_handler.sensor_log_sample_array[sensor_id][self.index_counter].timestamp, self.sensor_handler.sensor_log_sample_array[sensor_id][self.index_counter].trig_state.name)

            time.sleep(1/self.readout_frequency) # setting periodic time for sensor readout

            if(self.index_counter >= self.num_consecutive_trigs):
                self.start_stop_logging()
            
    
    def start_stop_logging(self):
        # add samples to consecutive_num_trigs_array
        for sensor_id in range(self.number_of_sensors):
            for list_index in range(self.num_consecutive_trigs):
                self.sensor_handler.consecutive_num_trigs_array[sensor_id][list_index] = self.sensor_handler.get_log_sample(sensor_id, self.index_counter - list_index)
        
        # print trig_state for all elements of the 2d-array
        for iy, ix in np.ndindex(self.sensor_handler.consecutive_num_trigs_array.shape):
            print(self.sensor_handler.consecutive_num_trigs_array[iy, ix].trig_state.name)

        # verify if all elements in each row have the same trig state
        row_check = np.all(self.sensor_handler.consecutive_num_trigs_array == self.sensor_handler.consecutive_num_trigs_array[:, [0]], axis=1)
        print("same_trig_result:", row_check)

        # verify that sensor trigs are stable (consecutive trigs have the same value)
        if(np.all(row_check) == True):
            if(self.current_state == AppLoggingState.INIT):
                self.current_state = AppLoggingState.IDLE
                print("current_state: ", self.current_state)

            elif(self.current_state == AppLoggingState.IDLE):
                for sensor_id in range(self.number_of_sensors):
                    # verify that any of the sensors are trigged to start the logging
                    if(self.sensor_handler.consecutive_num_trigs_array[sensor_id][0].trig_state.name == SensorTrigState.TRIG.name):
                        # start logging
                        self.current_state = AppLoggingState.LOGGING
                        print("current_state: ", self.current_state)
                        
                        # store start index of the active logging episode
                        self.index_log_start = self.index_counter
                        
                        # free up memory by clearing both arrays
                        #del self.sensor_handler.sensor_log_sample_array
                        #del self.sensor_handler.consecutive_num_trigs_array
                        # add the array creating part of code in sensor handler to a function that is called from here and in its init() function
                        break
            
            elif(self.current_state == AppLoggingState.LOGGING):
                # detect when to stop logging when both sensors are verified to be in a NO_TRIG state
                specific_value = SensorTrigState.NO_TRIG 

                # Extract the 'value' attribute
                attribute_values = np.vectorize(lambda obj: obj.trig_state)(self.sensor_handler.consecutive_num_trigs_array)

                # Check if all elements in each row match the specific value
                row_check = np.all(attribute_values == specific_value, axis=1)
                print("sensor_no_trig_result:", row_check)
                if(np.all(row_check) == True):
                    self.current_state = AppLoggingState.RESULT_EVALUATION
                    print("current_state: ", self.current_state)
                    # store stop index of the active logging episode
                    self.index_log_stop = self.index_counter
            
            elif(self.current_state == AppLoggingState.RESULT_EVALUATION):
                print(f"logged data at index: ({self.index_log_start, self.index_log_stop})")
                # create array with shape based on number of sensors and logged samples to store the TRIG timestamps
                timestamp_array = [[0 for _ in range(abs(self.index_log_start - self.index_log_stop))] for _ in range(self.number_of_sensors)]
                #print(timestamp_array)
                
                for sensor_id in range(self.number_of_sensors):
                    timestamp_array_index = 0
                    for list_index in range(self.index_log_start, self.index_log_stop):
                        if(self.sensor_handler.sensor_log_sample_array[sensor_id][list_index].trig_state.name == SensorTrigState.TRIG.name):
                            timestamp_array[sensor_id][timestamp_array_index] = self.sensor_handler.get_log_sample(sensor_id, list_index).timestamp
                        else:
                            timestamp_array[sensor_id][timestamp_array_index] = 0
                        # increase index
                        timestamp_array_index += 1
                print(timestamp_array)
                        
                result = self.get_sensor_timestamp_mean_value(timestamp_array)
                self.get_motion_direction(result)


    def get_sensor_timestamp_mean_value(self, timestamp_array):
        # create dict to store sensor mean time stamp value for each of the sensors
        sensor_mean_value_dict = dict()

        for sensor_id in range(self.number_of_sensors):
            sum = 0
            mean_value = 0
            valid_timestamp_counter = 0
            for value in timestamp_array[sensor_id]:
                if(value != 0):
                    valid_timestamp_counter += 1
                sum += value
            # resolve ZeroDivisionError
            if(valid_timestamp_counter != 0):
                mean_value = round(sum / valid_timestamp_counter)
            else:
                mean_value = 0
            # insert sensor id along with its mean timestamp in dict
            sensor_mean_value_dict["sensor" + str(sensor_id)] = mean_value
        # print dict
        for key, value in sensor_mean_value_dict.items():
            print(f"{key}: {sensor_mean_value_dict[key]}")

        return sensor_mean_value_dict


    def get_motion_direction(self, trig_dict):
        #IdentifiedMotionDirection
        
        # access dict with all sensors along with their timestamp mean values
        # sensor_trigs = {}
        # for key, value in args.items():
        #     print(key, value)
        #     sensor_trigs[key] = value

        # sort dictionary to determine which of the sensors that trigged first  sorted_dict = dict(sorted(my_dict.items()))
        # add if statement to determine which if an entry or exit was detected
        pass
    

def main():

    app = TrigEvaluationManager()
    app.run()


    # sensor_trig_threshold = 800     # sensor digital value (0 - 1023) to represent IR-sensor detection, below threshold value == sensor trig
    # number_of_sensors = 2
    # sensors = []
    # initial_num_sample_columns = 1
    # readout_frequency = 0.5     # Hz

    # for sensor_id in range(number_of_sensors):
    #     sensors.append(IrSensor(sensor_id, sensor_trig_threshold))
    
    # sensor_handler = SensorHandler(number_of_sensors, initial_num_sample_columns)

    
    # while(True):
    #     for sensor_id, sensor in enumerate(sensors):
    #         index_counter = sensor_handler.register_log_sample(sensor_id, *sensor.get_sensor_data())    # '*' unpacks the return tuple from function call)
    #         print(f"({int(sensor_id)}, {index_counter})")
    #         print(sensor_handler.sensor_log_sample_array[sensor_id][index_counter].value, sensor_handler.sensor_log_sample_array[sensor_id][index_counter].timestamp, sensor_handler.sensor_log_sample_array[sensor_id][index_counter].trig_state.name)

    #     time.sleep(1/readout_frequency) # setting periodic time for sensor readout



if __name__ == "__main__":
   main()