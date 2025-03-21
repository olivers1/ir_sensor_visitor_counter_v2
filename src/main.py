import numpy as np
from enum import Enum
from pathlib import Path
#from abc import ABC, abstractmethod
#from inspect import signature
from datetime import datetime
import os.path
from pathlib import Path

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
        self.create_log_arrays()  # create log arrays to store log samples

    def create_log_arrays(self):
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
    LOG_EVALUATION = 3


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
        self.readout_frequency = 12  # Hz [12 Hz - run] 
        self.index_counter = 0      # current index of sensor_log_sample_array
        self.num_consecutive_trigs = 5     # [5 - run] number of sensor trigs in a consecutive order to count it as a trig
        self.current_state = AppLoggingState.INIT
        self.index_log_start = 0
        self.index_log_stop = 0
        self.identified_motion_direction = IdentifiedMotionDirection.UNKNOWN
        self.app_logging_state = AppLoggingState.INIT   # initial app logging state
        self.sensor_handler = SensorHandler(self.number_of_sensors, self.initial_num_sample_columns, self.num_consecutive_trigs)
        self.sensor_first_trigged = None    # hold the sensor id that trigged first when the logging was initiated
        self.second_sensor_trigged_index = 0
        self.motion_direction_is_valid = False
        self.timestamp_array = None
        self.second_sensor_trig_timestamp_array = None

        self.valid_sensor_trigs = []
        for index in range(self.number_of_sensors):
            self.valid_sensor_trigs.append(False)

        # create or append to log file at specified path
        save_path = Path(__file__).resolve().parents[1]
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        filename = f"log_{timestamp}.txt"
        self.log_filename = os.path.join(save_path, filename)    
        

    def run(self):
        for sensor_id in range(self.number_of_sensors):
            self.sensors.append(IrSensor(sensor_id, self.sensor_trig_threshold))
    
        while(True):    
            for sensor_id, sensor in enumerate(self.sensors):
                self.index_counter = self.sensor_handler.register_log_sample(sensor_id, *sensor.get_sensor_data())    # '*' unpacks the return tuple from function call)
                #===
                print(f"(sensor_id, index_counter: {sensor_id}, {self.index_counter})") 
                print(self.sensor_handler.sensor_log_sample_array[sensor_id][self.index_counter].value, self.sensor_handler.sensor_log_sample_array[sensor_id][self.index_counter].timestamp, self.sensor_handler.sensor_log_sample_array[sensor_id][self.index_counter].trig_state.name)

            time.sleep(1/self.readout_frequency) # setting periodic time for sensor readout

            if(self.current_state == AppLoggingState.LOGGING):
                # print trig_state for all elements of the 2d-array
                #===
                for iy, ix in np.ndindex(self.sensor_handler.consecutive_num_trigs_array.shape):
                    print(self.sensor_handler.consecutive_num_trigs_array[iy, ix].trig_state.name)

            # start analysing logs after the consecutive_trigs array has been filled
            if(self.index_counter >= self.num_consecutive_trigs):
                self.start_stop_logging()
            
    
    def start_stop_logging(self):
        # add samples to consecutive_num_trigs_array
        for sensor_id in range(self.number_of_sensors):
            for list_index in range(self.num_consecutive_trigs):
                self.sensor_handler.consecutive_num_trigs_array[sensor_id][list_index] = self.sensor_handler.get_log_sample(sensor_id, self.index_counter - list_index)

        
        # check if trig state is stable by verifying that all elements in each row have the same trig state
        row_check = np.all(self.sensor_handler.consecutive_num_trigs_array == self.sensor_handler.consecutive_num_trigs_array[:, [0]], axis=1)
        #===
        print("same_trig_result:", row_check)
        #===
        print("current_state [before AppLoggingState]:", self.current_state.name)

        # verify that trig states are stable
        if(np.all(row_check) == True):
            # verify that sensors are unblocked and stable while in INIT state bofore going into IDLE
            if(self.current_state == AppLoggingState.INIT and self.sensor_handler.consecutive_num_trigs_array[sensor_id][0].trig_state.name == SensorTrigState.NO_TRIG.name):
                self.current_state = AppLoggingState.IDLE

            elif(self.current_state == AppLoggingState.IDLE):
                for sensor_id in range(self.number_of_sensors):
                    # verify that any of the sensors are trigged to start the logging
                    if(self.sensor_handler.consecutive_num_trigs_array[sensor_id][0].trig_state.name == SensorTrigState.TRIG.name):
                        # store the sensor id that triggered first
                        self.sensor_first_trigged = "sensor" + str(sensor_id)
                        #===
                        print("sensor_first_trigged:", self.sensor_first_trigged)
                        # start logging
                        self.current_state = AppLoggingState.LOGGING
                        
                        # store start index of the active logging episode
                        self.index_log_start = self.index_counter

                        # write to list that contains details of which of the all sensors that have had valid trigs
                        self.valid_sensor_trigs[sensor_id] = True
                        break
                print("valid_sensor_trigs:", self.valid_sensor_trigs)
            
            elif(self.current_state == AppLoggingState.LOGGING):
                print("current_state [current]:", self.current_state.name)
                for sensor_id, sensor in enumerate(self.sensors):
                    print(f"(sensor_id, index_counter: {sensor_id}, {self.index_counter})")
                    print(self.sensor_handler.sensor_log_sample_array[sensor_id][self.index_counter].value, self.sensor_handler.sensor_log_sample_array[sensor_id][self.index_counter].timestamp, self.sensor_handler.sensor_log_sample_array[sensor_id][self.index_counter].trig_state.name)

                for sensor_id in range(self.number_of_sensors):
                    if(self.sensor_handler.consecutive_num_trigs_array[sensor_id][0].trig_state.name == SensorTrigState.TRIG.name):
                        # logging trig state by popluating list for each sensor with valid trig states (num consecutive trigs)
                        self.valid_sensor_trigs[sensor_id] = True
                
                # detect when to stop logging when both sensors are verified to be in a NO_TRIG state
                specific_value = SensorTrigState.NO_TRIG 

                # Extract the 'value' attribute
                attribute_values = np.vectorize(lambda obj: obj.trig_state)(self.sensor_handler.consecutive_num_trigs_array)

                # Check if all elements in each row match the specific value, thus is not trigged
                row_check = np.all(attribute_values == specific_value, axis=1)
                print("sensor_no_trig_result:", row_check)
                if(np.all(row_check) == True):
                    # start logging
                    self.current_state = AppLoggingState.LOG_EVALUATION
                    # store stop index of the active logging episode
                    self.index_log_stop = self.index_counter
            
            elif(self.current_state == AppLoggingState.LOG_EVALUATION):
                print("current_state [current]:", self.current_state.name)
                print(f"logged data at index: ({self.index_log_start, self.index_log_stop})")
                print("valid_sensor_trigs:", self.valid_sensor_trigs)
                # create array with shape based on number of sensors and logged samples to store the TRIG timestamps
                self.timestamp_array = [[0 for _ in range(abs(self.index_log_start - self.index_log_stop))] for _ in range(self.number_of_sensors)]
                #print(timestamp_array)
                
                for sensor_id in range(self.number_of_sensors):
                    timestamp_array_index = 0
                    for list_index in range(self.index_log_start, self.index_log_stop):
                        if(self.sensor_handler.sensor_log_sample_array[sensor_id][list_index].trig_state.name == SensorTrigState.TRIG.name):
                            self.timestamp_array[sensor_id][timestamp_array_index] = self.sensor_handler.get_log_sample(sensor_id, list_index).timestamp
                        else:
                            self.timestamp_array[sensor_id][timestamp_array_index] = 0

                        # increase index
                        timestamp_array_index += 1
                        
                print(self.timestamp_array)

                # evaluate sensor logs     
                self.get_sensor_statistics()
                self.validate_motion_direction()
                self.get_motion_direction()
                # clear sensor log
                self.clean_log_memory()
            
            #===
            print("valid_sensor_trigs:", self.valid_sensor_trigs)
            #===
            print("current_state [after AppLoggingState]:", self.current_state.name)


    def get_sensor_statistics(self):
        # create dict to store sensor mean time stamp value for each of the sensors
        self.sensor_mean_value_dict = dict()

        # create a dict to store number of trig states for each of the sensors
        self.sensor_number_of_trigs = dict()

        for sensor_id in range(self.number_of_sensors):
            sum = 0
            mean_value = 0
            valid_timestamp_counter = 0
            for value in self.timestamp_array[sensor_id]:
                if(value != 0):
                    valid_timestamp_counter += 1
                sum += value
            # resolve ZeroDivisionError
            if(valid_timestamp_counter != 0):
                mean_value = round(sum / valid_timestamp_counter)
            else:
                mean_value = 0
            # insert sensor id along with its mean timestamp in dict
            self.sensor_mean_value_dict["sensor" + str(sensor_id)] = mean_value
            # insert sensor id along with number of trig states
            self.sensor_number_of_trigs["sensor" + str(sensor_id)] = valid_timestamp_counter
        # print dict
        print("sensor_mean_value_dict:")
        for key, value in self.sensor_mean_value_dict.items():
            print(f"{key}: {self.sensor_mean_value_dict[key]}")
        
        print("sensor_number_of_trigs:")
        for key, value in self.sensor_number_of_trigs.items():
            print(f"{key}: {self.sensor_number_of_trigs[key]}")


    def validate_motion_direction(self):
        # validate the identified motion direction based on which of the sensors that trigged first and 
        # which of the them that triggered last (highest timestamp) 

        self.motion_direction_is_valid = False
        if(self.sensor_first_trigged == "sensor0"):
            if(self.sensor_mean_value_dict["sensor0"] < self.sensor_mean_value_dict["sensor1"]):
                self.motion_direction_is_valid = True
        elif(self.sensor_first_trigged == "sensor1"):
            if(self.sensor_mean_value_dict["sensor1"] < self.sensor_mean_value_dict["sensor0"]):
                self.motion_direction_is_valid = True
        

    def write_to_log_file(self, trig_dict, motion_mean_value):  
        # write to log file
        with open(self.log_filename, "a") as f:
            f.write('{} : {}\n'.format(str(motion_mean_value), str(self.identified_motion_direction.name)))
            for key, value in trig_dict.items():
                f.write('  {} : {}\n'.format(key,  str(trig_dict[key])))


    def get_motion_direction(self):
        # verify that both sensors have been trigged and that motion direction is valid otherwise motion is classified as UNKNOWN
        if(all(self.valid_sensor_trigs) == True and self.motion_direction_is_valid == True):
            if(self.sensor_mean_value_dict["sensor0"] > 0 and self.sensor_mean_value_dict["sensor1"] > 0):
                # check which sensor that trigged first
                if(self.sensor_mean_value_dict["sensor0"] < self.sensor_mean_value_dict["sensor1"]):
                    self.identified_motion_direction = IdentifiedMotionDirection.EXIT
                elif(self.sensor_mean_value_dict["sensor0"] > self.sensor_mean_value_dict["sensor1"]):
                    self.identified_motion_direction = IdentifiedMotionDirection.ENTRY
        else:
            self.identified_motion_direction = IdentifiedMotionDirection.UNKNOWN

        print("identified_motion_direction:", self.identified_motion_direction.name)

        # get summarized mean timestamp value from dictionary that only contains valid (!= 0) timestamps
        mean_value = 0
        num_values = 0
        for key, value in self.sensor_mean_value_dict.items():
            # verifiy that timestamps are valid when calculating average timestamp value
            if(self.sensor_mean_value_dict[key] != 0):
                mean_value += self.sensor_mean_value_dict[key]
                num_values += 1
        
        motion_mean_value = round(mean_value / num_values)

        # write to log file
        self.write_to_log_file(self.sensor_mean_value_dict, motion_mean_value)

        self.current_state = AppLoggingState.IDLE
        print("current_state:", self.current_state.name)

    def clean_log_memory(self):
        # reset logging state and logged parameters
        self.index_log_start = 0
        self.index_log_stop = 0

        # free up memory by clearing both arrays
        print("log memory cleared")
        del self.sensor_handler.sensor_log_sample_array
        del self.sensor_handler.consecutive_num_trigs_array
        self.sensor_handler.index_counter = 0
        self.sensor_handler.create_log_arrays()
        self.valid_sensor_trigs = [False for _ in self.valid_sensor_trigs]
        self.sensor_first_trigged = None

    

def main():

    app = TrigEvaluationManager()
    app.run()

if __name__ == "__main__":
   main()