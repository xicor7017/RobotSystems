# from ezblock import Servo,PWM,fileDB,Pin,ADC

try:
    from ezblock import *
    from ezblock import __reset_mcu__
    from servo import Servo 
    from pwm import PWM
    from pin import Pin
    from adc import ADC
    from filedb import fileDB
    import time
    __reset_mcu__ ()
    time.sleep (0.01)
except ImportError:
    print ("This computer does not appear to be a PiCar -X system (ezblock is not present). Shadowing hardware calls with substitute functions ")
    from sim_ezblock import *

from filedb import fileDB
import logging
from logdecorator import log_on_start , log_on_end , log_on_error
import atexit

logging_format = '%(asctime)s %(message)s'
logging.basicConfig(format=logging_format , level=logging.INFO ,datefmt ="%H:%M:%S")
logging.getLogger ().setLevel(logging.DEBUG)

class Picarx(object):
    PERIOD = 4095
    PRESCALER = 10
    TIMEOUT = 0.02

    @log_on_start(logging.DEBUG , "Main class instantiated")
    def __init__(self):
        #Making sure that the motors stop when the program execution stops.
        atexit.register(self.cleanup)

        self.dir_servo_pin = Servo(PWM('P2'))
        self.camera_servo_pin1 = Servo(PWM('P0'))
        self.camera_servo_pin2 = Servo(PWM('P1'))
        try:
            self.config_file = fileDB('/home/pi/.config')
            self.dir_cal_value = int(self.config_file.get("picarx_dir_servo", default_value=0))
            self.cam_cal_value_1 = int(self.config_file.get("picarx_cam1_servo", default_value=0))
            self.cam_cal_value_2 = int(self.config_file.get("picarx_cam2_servo", default_value=0))
        except FileNotFoundError:
            logging.debug("Exception encountered")
            self.dir_cal_value = 0
            self.cam_cal_value_1 = 0
            self.cam_cal_value_2 = 0
        
        self.dir_servo_pin.angle(self.dir_cal_value)
        self.camera_servo_pin1.angle(self.cam_cal_value_1)
        self.camera_servo_pin2.angle(self.cam_cal_value_2)

        self.left_rear_pwm_pin = PWM("P13")
        self.right_rear_pwm_pin = PWM("P12")
        self.left_rear_dir_pin = Pin("D4")
        self.right_rear_dir_pin = Pin("D5")


        self.S0 = ADC('A0')
        self.S1 = ADC('A1')
        self.S2 = ADC('A2')

        self.motor_direction_pins = [self.left_rear_dir_pin, self.right_rear_dir_pin]
        self.motor_speed_pins = [self.left_rear_pwm_pin, self.right_rear_pwm_pin]
        try:
            self.cali_dir_value = self.config_file.get("picarx_dir_motor", default_value="[1,1]")
            self.cali_dir_value = [int(i.strip()) for i in self.cali_dir_value.strip("[]").split(",")]
        except FileNotFoundError:
            self.cali_dir_value = [0.0, 0.1]
        
        self.cali_speed_value = [0, 0]
        self.dir_current_angle = 0
        #初始化PWM引脚
        for pin in self.motor_speed_pins:
            pin.period(self.PERIOD)
            pin.prescaler(self.PRESCALER)

    def cleanup(self):
        self.stop()

    def set_motor_speed(self,motor,speed):
        # global cali_speed_value,cali_dir_value
        motor -= 1
        if speed >= 0:
            direction = 1 * self.cali_dir_value[motor]
        elif speed < 0:
            direction = -1 * self.cali_dir_value[motor]
        speed = abs(speed)
        if speed != 0:
            speed = int(speed /2 ) + 50
        speed = speed - self.cali_speed_value[motor]
        if direction < 0:
            self.motor_direction_pins[motor].high()
            self.motor_speed_pins[motor].pulse_width_percent(speed)
        else:
            self.motor_direction_pins[motor].low()
            self.motor_speed_pins[motor].pulse_width_percent(speed)

    def motor_speed_calibration(self,value):
        # global cali_speed_value,cali_dir_value
        self.cali_speed_value = value
        if value < 0:
            self.cali_speed_value[0] = 0
            self.cali_speed_value[1] = abs(self.cali_speed_value)
        else:
            self.cali_speed_value[0] = abs(self.cali_speed_value)
            self.cali_speed_value[1] = 0

    def motor_direction_calibration(self,motor, value):
        # 0: positive direction
        # 1:negative direction
        # global cali_dir_value
        motor -= 1
        if value == 1:
            self.cali_dir_value[motor] = -1 * self.cali_dir_value[motor]
        self.config_file.set("picarx_dir_motor", self.cali_dir_value)


    def dir_servo_angle_calibration(self,value):
        # global dir_cal_value
        self.dir_cal_value = value
        print("calibrationdir_cal_value:",self.dir_cal_value)
        self.config_file.set("picarx_dir_servo", "%s"%value)
        self.dir_servo_pin.angle(value)

    def set_dir_servo_angle(self,value):
        # global dir_cal_value
        self.dir_current_angle = value
        angle_value  = value + self.dir_cal_value
        print("angle_value:",angle_value)
        # print("set_dir_servo_angle_1:",angle_value)
        # print("set_dir_servo_angle_2:",dir_cal_value)
        self.dir_servo_pin.angle(angle_value)

    def camera_servo1_angle_calibration(self,value):
        # global cam_cal_value_1
        self.cam_cal_value_1 = value
        self.config_file.set("picarx_cam1_servo", "%s"%value)
        print("cam_cal_value_1:",self.cam_cal_value_1)
        self.camera_servo_pin1.angle(value)

    def camera_servo2_angle_calibration(self,value):
        # global cam_cal_value_2
        self.cam_cal_value_2 = value
        self.config_file.set("picarx_cam2_servo", "%s"%value)
        print("picarx_cam2_servo:",self.cam_cal_value_2)
        self.camera_servo_pin2.angle(value)

    def set_camera_servo1_angle(self,value):
        # global cam_cal_value_1
        self.camera_servo_pin1.angle(-1*(value + -1*self.cam_cal_value_1))
        # print("self.cam_cal_value_1:",self.cam_cal_value_1)
        print((value + self.cam_cal_value_1))

    def set_camera_servo2_angle(self,value):
        # global cam_cal_value_2
        self.camera_servo_pin2.angle(-1*(value + -1*self.cam_cal_value_2))
        # print("self.cam_cal_value_2:",self.cam_cal_value_2)
        print((value + self.cam_cal_value_2))

    def get_adc_value(self):
        adc_value_list = []
        adc_value_list.append(self.S0.read())
        adc_value_list.append(self.S1.read())
        adc_value_list.append(self.S2.read())
        return adc_value_list

    def set_power(self,speed):
        self.set_motor_speed(1, speed)
        self.set_motor_speed(2, speed) 

    def backward(self,speed):
        current_angle = self.dir_current_angle
        if current_angle != 0:
            abs_current_angle = abs(current_angle)
            # if abs_current_angle >= 0:
            if abs_current_angle > 40:
                abs_current_angle = 40
            power_scale = (100 - abs_current_angle) / 100.0 
            print("power_scale:",power_scale)
            if (current_angle / abs_current_angle) > 0:
                self.set_motor_speed(1, -1*speed)
                self.set_motor_speed(2, speed * power_scale)
            else:
                self.set_motor_speed(1, -1*speed * power_scale)
                self.set_motor_speed(2, speed )
        else:
            self.set_motor_speed(1, -1*speed)
            self.set_motor_speed(2, speed)  

    def forward(self,speed):
        current_angle = self.dir_current_angle
        if current_angle != 0:
            abs_current_angle = abs(current_angle)
            # if abs_current_angle >= 0:
            if abs_current_angle > 40:
                abs_current_angle = 40
            power_scale = (100 - abs_current_angle) / 100.0 
            
            if (current_angle / abs_current_angle) > 0:
                self.set_motor_speed(1, speed)
                self.set_motor_speed(2, -1*speed * power_scale)
            else:
                self.set_motor_speed(1, speed * power_scale)
                self.set_motor_speed(2, -1*speed )
        else:
            self.set_motor_speed(1, speed)
            self.set_motor_speed(2, -1*speed)                  

    def stop(self):
        self.set_motor_speed(1, 0)
        self.set_motor_speed(2, 0)


    def Get_distance(self):
        timeout=0.01
        trig = Pin('D8')
        echo = Pin('D9')

        trig.low()
        time.sleep(0.01)
        trig.high()
        time.sleep(0.000015)
        trig.low()
        pulse_end = 0
        pulse_start = 0
        timeout_start = time.time()
        while echo.value()==0:
            pulse_start = time.time()
            if pulse_start - timeout_start > timeout:
                return -1
        while echo.value()==1:
            pulse_end = time.time()
            if pulse_end - timeout_start > timeout:
                return -2
        during = pulse_end - pulse_start
        cm = round(during * 340 / 2 * 100, 2)
        #print(cm)
        return cm

    def discrete_forward_action(self, speed, direction=0, time=2):
        '''To drive the car forward for fixed amount of time'''
        t_start = time.time()
        t_current = time.time()
        self.set_dir_servo_angle(direction)
        while t_current < t_start + time:   #For the requested amount of time
            self.forward(speed)
            time.sleep(0.1)
    
    def discrete_backward_action(self, speed, direction=0, time=2):
        '''To drive the car forward for fixed amount of time'''
        t_start = time.time()
        t_current = time.time()
        self.set_dir_servo_angle(direction)
        while t_current < t_start + time:   #For the requested amount of time
            self.backward(speed)
            time.sleep(0.1)

    def parallel_park(self, speed, side="left"):
        if side == "left":
            direction = 30
        else:
            direction = -30

        self.discrete_backward_action(speed, direction=direction, time=0.5)
        self.discrete_forward_action(speed/2, direction=0, time=0.2)

    def k_turn(self, speed, side='left'):
        if side == 'left':
            direction = 60
        else:
            direction = -60

        self.discrete_forward_action(speed, direction=direction, time=0.6)
        self.discrete_backward_action(speed/2, direction=direction, time=0.4)
        self.discrete_forward_action(speed, direction=-direction, time=0.6)

    def user_control(self, speed):
        continue_loop = True
        side = "left"
        while continue_loop:
            user_input = input()
            
            if user_input == "x":              #Exit condition
                continue_loop=False

            elif user_input == "l":            #Change side of actions
                side = "left"
            
            elif user_input == "r":            #Change side of actions
                side = "right"
                 
            elif user_input == 'k':            #K point turn
                self.k_turn(speed, side=side)

            elif user_input == 'p':             #Parallel_parking
                self.parallel_park(speed, side=side)
            
            elif user_input == 'f':             #Forward 
                self.discrete_forward_action(speed)

            elif user_input == "b":             #Backward
                self.discrete_backward_action(speed)

            else:
                logging.debug("Invalid user input. Available commands: \n x = exit \n l = change to left side  \n r = change to right side  \n k = K point turn \n p = parallel parking manuever  \n f = forward \n b = backward ")


if __name__ == "__main__":
    px = Picarx()
    px.forward(50)
    time.sleep(1)
    px.stop()
    # set_dir_servo_angle(0)
    # time.sleep(1)
    # self.set_motor_speed(1, 1)
    # self.set_motor_speed(2, 1)
    # camera_servo_pin.angle(0)
# set_camera_servo1_angle(cam_cal_value_1)
# set_camera_servo2_angle(cam_cal_value_2)
# set_dir_servo_angle(dir_cal_value)

# if __name__ == "__main__":
#     try:
#         # dir_servo_angle_calibration(-10) 
#         while 1:
#             test()
#     finally: 
#         stop()