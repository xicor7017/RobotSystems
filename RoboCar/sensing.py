import time
from adc import ADC
from picarx_improved import Picarx

class Sensing:

    def __init__(self):
        self.left_adc = ADC(0)
        self.center_adc = ADC(1)
        self.right_adc = ADC(2)

    def read(self):
        values = [self.left_adc.read(), self.center_adc.read(), self.right_adc.read()]
        return values

class Interpretation:
    ##
    # This class works by discretizing data which makes interpretations easier
    ##
    def __init__(self, sensitivity=0.5, polarity=0):
        self.sensitivity = sensitivity
        self.polarity = polarity

    def process(self, adc_values):
        mean_adc_value = sum(adc_values)/3

        edge_left = int(abs(adc_values[1] - adc_values[0]) > self.sensitivity)
        edge_right = int(abs(adc_values[1] - adc_values[2]) > self.sensitivity)

        #Steer 0.0  : No turning required
        #Steer 1.0  : Left turning required
        #Steer -1.0 : Right turning required
        if edge_left and adc_values[1] > adc_values[0]:
            self.steer = 1.0
            if not self.polarity:
                self.steer = -1.0
        elif edge_left and adc_values[1] < adc_values[0]:
            self.steer = -1.0
            if not self.polarity:
                self.steer = 1.0

        elif edge_right and adc_values[2] > adc_values[1]:
            self.steer = 1.0
            if not self.polarity:
                self.steer = -1.0
        
        elif edge_right and adc_values[2] < adc_values[1]:
            self.steer = -1.0
            if not self.polarity:
                self.steer = 1.0

        else:   #Well aligned
            self.steer = 0.0

    def output(self, adc_values):
        self.process(adc_values)
        robot_position = -1.0 * self.steer   #Robot position is opposite to the required steer direction
        return robot_position

class Controller:

    def __init__(self, scaling_factor=1.0):
        self.scaling_factor = scaling_factor
        self.px = Picarx()

    def control(self, robot_position):
        steering_angle = self.scaling_factor * (-1.0 * robot_position)
        self.px.set_dir_servo_angle(steering_angle)

        return steering_angle

class Main:
    #The control loop class

    def __init__(self):
        self.sensing = Sensing()
        self.Interpreter = Interpretation()
        self.controller = Controller()

        self.delay = 0.05
        self.speed = 0.5

    def control_loop(self):
        while True:
            adc_values = self.sensing.read()
            robot_position = self.output(adc_values)

            self.controller.control(robot_position)
            self.controller.px.forward(speed = self.speed)
            time.sleep(self.delay)