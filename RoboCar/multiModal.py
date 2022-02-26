import time, logging
from picarx_improved import Picarx
from threading import Event

from rossros import *
from opencv_lane import *

try:
    from servo import Servo
    from pwm import PWM
    from pin import Pin
    from adc import ADC
    from filedb import fileDB
    import time
    from ultrasonic import Ultrasonic
    from utils import reset_mcu
    reset_mcu()
    time.sleep(0.01)
except:
    print("Shadowing hardware as running simulation")
    time.sleep(1)
    from sim_ezblock import*

stop_rqt = Event()
vid=cv2.VideoCapture(0)

class ultrasonic_Sensor:
    def __init__(self):
        self.D0 = Pin("D0")
        self.D1 = Pin("D1")

    def get_ultrasensor_reading(self):
        return Ultrasonic(self.D0, self.D1).read()
        
class ultrasonic_Interpreter:
    def __init__(self, threshold = 15):
        self.threshold = threshold
       
    def interpret_obstacle(self, distance):
        if distance <= self.threshold:  #Stopping criterion
            return True
        else:
            return False

class ultrasonic_Controller:
    def __init__(self, speed=35):
        self.speed = speed

    def ultra_controller(self, car, stop):
        if stop:
            car.stop()
        else:
            car.forward(self.speed)

class Sensing(object):
    def __init__(self,ref = 1000):
        self.chn_0 = ADC("A0")
        self.chn_1 = ADC("A1")
        self.chn_2 = ADC("A2")
        self.ref = ref

    def get_grayscale_data(self):
        adc_value_list = []
        adc_value_list.append(self.chn_0.read())
        adc_value_list.append(self.chn_1.read())
        adc_value_list.append(self.chn_2.read())
        return adc_value_list

class Interpreter(object):
    def __init__(self, sensitivity, reverse=False):
        self.sensitivity = sensitivity
        self.polarity = 1
        if reverse:
            self.polarity=0

    def interpret_position(self, fl):
        if abs(fl[0] - fl[2]) < self.sensitivity:
            if fl[2]+abs((fl[2]-fl[0])/4) < fl[1]:
                pos = -1 * self.polarity   
            else:
                pos = -.5 * self.polarity
        elif fl[0] < fl[2]:
                if fl[0] + abs((fl[2]-fl[0])/4) > fl[1]:
                    pos = .5 * self.polarity   
                else:
                    pos = 1 * self.polarity
        else:
            pos = 0
        return pos

class Controller():
    def __init__(self,px, scaling=35):
        self.scaling = scaling
        self.px = px

    def controller(self,pos):
        try:
            self.px.set_dir_servo_angle(pos*self.scaling)
            self.px.forward(30)
            time.sleep(0.05)
        finally:
            self.px.stop()
                
    def camera_control(self,frame):
       lane_lines=detect_lane(frame)
       frame_shape=frame.shape
       angle,lines = calculate_heading(lane_lines,frame_shape[1],frame_shape[0])
       angle_to_mid_deg = int((angle*180)/3.14)
       self.px.set_dir_servo_angle(angle_to_mid_deg*0.7)
       self.px.forward(20)
       time.sleep(0.1)

def sigint_handler(sig, frame):
    global stop_rqt
    stop_rqt.set()


if __name__ == "__main__":

    #Setting up parameters
    sensitivity = 300
    scale = 120
    runtime = 40
    speed = 5
    thresh = 20
    sensor_delay = 0.05
    interpreter_delay = 0.05
    control_delay = 0.05

    #Setting up ultrasonic busses
    us_values_bus = Bus(initial_message=0,
                           name="ultrasonic sensor")
    us_interpreter_bus = Bus(initial_message=False,
                                name="ultrasonic interpreter")

                                #grayscale
    sensor_values_bus = Bus(initial_message=[0, 0, 0],
                            name="sensor values")
    sensor_interpreter_bus = Bus(initial_message=0,
                          name="sensor interpreter")

    car = Picarx()

    #line following
    sensor = Sensing()
    interpreter = Interpreter(sensitivity, False)
    control = Controller(car, scale)

    us_sensor = ultrasonic_Sensor()
    us_interpreter = ultrasonic_Interpreter(thresh=thresh)
    us_control = ultrasonic_Controller(speed = speed)

    #grayscale
    greyscale_read = Producer(sensor.get_grayscale_data(),
                              output_busses=sensor_values_bus,
                              delay=0.09,
                              name="Greyscale Sensor")

    greyscale_proc = ConsumerProducer(interpreter.interpret_position,
                                      input_busses=sensor_values_bus,
                                      output_busses=sensor_interpreter_bus,
                                      delay=0.1,
                                      name="Greyscale Sensor interpretation")
    greyscale_cont = Consumer(control.controller,
                          input_busses=sensor_interpreter_bus,
                          delay=0.1,
                          name="Greyscale Steering Controller")

    us_read = Producer(us_sensor.get_ultrasensor_reading,
                              output_busses=us_values_bus,
                              delay=0.09,
                              name="Ultrasonic Sensor")

    us_proc = ConsumerProducer(us_interpreter.interpret_obstacle,
                                      input_busses=us_values_bus,
                                      output_busses=us_interpreter_bus,
                                      delay=0.1,
                                      name="Ultrasonic Sensor inter")
    us_cont = Consumer(us_control.ultra_controller,
                              input_busses=us_interpreter_bus,
                              delay=0.1,
                              name="Ultrasonic Steering control")

    #Prepraring concurrent threads
    thread_list = [greyscale_read, greyscale_proc, greyscale_cont, us_read, us_proc, us_read]

    #Starting execution
    runConcurrently(thread_list)

    car.stop()