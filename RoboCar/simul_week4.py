#!/usr/bin/python3

import time
import concurrent.futures
from threading import Lock
from sensing import Sensing, Interpretation, Controller

try :
    from ezblock import __reset_mcu__
    from ezblock import *
    
    __reset_mcu__()
    time.sleep(0.01)
except ImportError :
    print ("Simulator")
    from sim_ezblock import *


import logging
logging_format = "%(asctime) s : %(message) s " 
logging.basicConfig(level = logging.INFO) #format = logging_format, level = logging.INFO, )#datefmt ="% H :% M :% S ")

# logging.getLogger().setLevel(logging.DEBUG) 
# comment out this line to remove debugging comments
# logging.debug (message) Use this line to print debugging info

from logdecorator import log_on_start , log_on_end , log_on_error 

class DBus:
    '''
    Class for passing msgs
    '''
    def __init__(self):
        self.msg = 0
        
    def read(self):
        return self.msg
    
    def write(self, msg):
        self.msg = msg
    
    def testBus(self):
        print("Before writing: ", self.read())
        self.write('Testing read/write')
        print("After writing: ", self.read())


def sensor_msg_prod(s, bus, delay):
    lock = Lock()
    while True:
        with lock:
            adcs = s.get_adc_value()
        bus.write(adcs)
        time.sleep(delay)
        
    
def interpreter(i, input_bus, output_bus, delay):
    while True:
        logging.info("input_bus: {0}".format(input_bus.read()))
        if input_bus.read() != None:
            position = i.get_grayscale_value(input_bus.read())
            output_bus.write(position)
            logging.info("output_bus: {0}".format(output_bus.read()))
            time.sleep(delay)
        else:
            time.sleep(delay)
    
def controller_msg_consumer(c, out_bus, delay, speed):
    while True:
        if out_bus.read() != None:
            c.line_following(out_bus.read(), speed)
            time.sleep(delay)   #Delay is required for stability
        else:
            time.sleep(delay)
            
def simultaneous(m,s,i,c, speed):
    #Function to run independent threaded operations
    sensor_delay = 0.2
    interpreter_delay = 0.2
    controller_delay = 0.2
    in_bus = DBus()
    out_bus = DBus()
    
    with concurrent.futures.ThreadPoolExecutor(max_workers=3) as executor:
        bus_sensor = executor.submit(sensor_msg_prod, s, in_bus, sensor_delay)
        bus_interpreter = executor.submit(interpreter, i, in_bus, out_bus, interpreter_delay)
        bus_controller = executor.submit(controller_msg_consumer, c, out_bus, controller_delay, speed)
    
    logging.info("Threads started")
    logging.info(bus_controller)
    bus_sensor.result()
    bus_interpreter.result()
    bus_controller.result()

        
if __name__ == "__main__":
    b = DBus()
    b.test()