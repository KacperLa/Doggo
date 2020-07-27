import json
import math
import struct
import copy
import numpy as np
import time
import odrive
from odrive.utils import dump_errors as d_e
from odrive.enums import *


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float32
from std_srvs.srv import Trigger


packet = {
        'doggo': {
            'right': {
                'x' : 0,
                'y' : 0,
            },
            'left': {
                'x' : 0,
                'y' : 0,
                }
        },
    }

import math
import os
import json

class Axis(object):
    """
    Dummy Axis Object
    Each axis no matter its type depends on this
    framework to function.
    When a new axis is created using the axis object
    applicable functions are replaced with relevant functions
    specific to that axis
    """

    def __init__(self):
        print('Creating Axis: ', str(self))

    def connect(self):
        pass

    def start(self):
        pass

    def run(self):
        pass

    def e_stop(self):
        pass

    def stop(self):
        pass

    def set_settings(self):
        pass

    def get_settings(self):
        pass

    def set_commands(self):
        pass

    def enable(self):
        pass

    def disable(self):
        pass

    def dump_errors(self):
        pass


class Dolly(Axis):
    """
    Dolly Class: inheres its structure from the Axis Object
    Responsible for handling the function of its three
    sub axes( three drive modules)
    """
    def __init__(self, config):
        """
        Initialization of the Dolly Class:
        define class variables:
        enable: what is this !!!!!!
        set points: variables set by the set commands function used in the run function
        :param
        config: JSON file with parameters relevant to the initialized axes
        """
       
        self.enabled = config['enabled']

                #  Create three Modules using the imported config file
        self.modules = {
            'back': Module('right', config['right'], self.enabled),
            'left': Module('left', config['left'], self.enabled),
            #'right': Module(self.bus, 'right', config['right'], self.enabled),
        }

        for key, module in self.modules.items():
            module.connect()

    def enable(self):
        """
        Overwrites inherited function from axis object
        Run .enable function for each Module(axis)
        """
        # self.enabled = True
        for key, module in self.modules.items():
            module.enable()

    def disable(self):
        """
        Overwrites inherited function from axis object
        Run .disable function for each Module(axis)
        """
        # self.enabled = False
        for key, module in self.modules.items():
            module.disable()
            
    def home(self):
        """
        Overwrites inherited function from axis object
        Run .home function for each Module(axis)
        """
        # self.enabled = False
        for key, module in self.modules.items():
            module.home()

    def calibrate_encoders(self):
        """
        Overwrites inherited function from axis object
        Run .home function for each Module(axis)
        """
        # self.enabled = False
        for key, module in self.modules.items():
            module.calibrate_encoders()

    def reboot(self):
        """
        Overwrites inherited function from axis object
        Run .home function for each Module(axis)
        """
        # self.enabled = False
        for key, module in self.modules.items():
            module.reboot()

    def check_state(self):
        dolly_state = []
        
        for key, module in self.modules.items():
            dolly_state.extend(module.state)
            
        return np.array(dolly_state)
    
    def get_vin(self):
        """
        Pull bus voltage from the back module
        :return: buss voltage
        """
        if self.enabled:
            return self.modules['back'].get_vin()

    def set_commands(self, commands):
        """
        Overwrite inherited function
        set set point class variables
        and call run class
        :param commands: JSON object
        """
        self.right_setpoint = commands['right']
        self.left_setpoint = commands['left']


    def run(self):
        """
        Overwrite inherited function
        Calculate angle and velocity for each module
        Pass these values on to each module
        call run for each module(axis)
        """
        right_angle, right_gamma = self.inverse_kinematics(self.right_setpoint['x'], self.right_setpoint['y'])
        left_angle, left_gamma = self.inverse_kinematics(self.left_setpoint['x'], self.left_setpoint['y'])


        if self.enabled:
            self.modules['left'].set_commands(left_angle, left_gamma)
            #self.modules['right'].set_commands(angle_2, v_2)
            self.modules['back'].set_commands(right_angle, right_gamma)

            for key, module in self.modules.items():
                module.run()
                
    def request_pos(self):
        for key, module in self.modules.items():
            module.request_pos()

    def update_pos(self, data):
        for key, module in self.modules.items():
            module.update_pos(data)
            
    def update_state(self, data):
        for key, module in self.modules.items():
            module.update_state(data)

    def inverse_kinematics(self, x, y):
        """
        This kinematic model is a copy of the kinematic model from the PC_UI
        any changes made here should be replicated in the PC_UI kinematic
        model to ensure that the dolly behaves as expected
        :param theta: direction of dolly's velocity vector in dolly coordinate system
        :param w: magnitude of dolly's rotational velocity
        :param v: magnitude of dolly's velocity
        :return: angle and velocity for each module
        """
        upper = .09
        lower = .162
        
        dis = [x, y]
        #print("dis: ", dis, "target: ", target, "orgin: ", orgin)
        r = np.sqrt(np.sum(np.multiply(dis, dis)))
        r_lim = upper+lower
        if r > r_lim:
            r = r_lim
        
        angle = math.atan2(*dis)
        
        #print("upper: ", upper, "r: ", r, "lower: ", lower)
        cos_param = ((upper**2) + (r**2) - (lower**2))/(2.0*upper*r)
        if cos_param < -1.0:
            print("r is to small to find valid gamma value")
            gamma = math.pi
        elif cos_param > 1.0:
            print("r is to large to find a valid gamma value")
            gamma = 0
        else:
            gamma = math.acos(cos_param)
        
        return angle, gamma

class Module:
    """
    Class responsible for handling the function of the ODrive
    """
    def __init__(self, name, config, enabled):
        """
        Initialise ODrive using given config JSON object
        :param config: Properties specific to given module
        :param enabled:
        """
        self.enabled = enabled
        self.connected = False
        self.state = [0, 0]

        # ODrive
        self.module_name = name
        self.odrive = None
        print(config)
        self.odrive_serial_number = config['odrive_serial_number']
        
        # constants
        self.front_cprad = (2000*4.5)/(-math.pi*2)
        self.back_cprad = (2000*4.5)/(-math.pi*2)


        # calibration variables
        self.offset_front = 0
        self.offset_back = 0
        self.pos_estimate_front = 0
        self.pos_estimate_back = 0
        
        # set points
        self.back_setpoint = None
        self.front_setpoint = None
        self.back_n = None
        self.front_n = None
        
    def connect(self):
        print('starting module', self.module_name, self.enabled)
        
        if self.enabled:
            self.odrive = odrive.find_any(serial_number=str(self.odrive_serial_number))
            print(f'{self.module_name} ODrive found')
            # put all axes in idle state
            self.connected = True
            self.disable()    
    
    def request_pos(self):
        if self.enabled and self.connected:
            self.front_n = (self.odrive.axis0.encoder.pos_estimate - self.offset_front + ((math.pi/2)*self.front_cprad)) / self.front_cprad
            self.back_n =  (self.odrive.axis1.encoder.pos_estimate - self.offset_back - ((math.pi/2)*self.back_cprad)) / self.back_cprad

    def check_state(self, data):
        if self.enabled and self.connected:
            self.state[0] = self.odrive.axis0.current_state
            self.state[1] = self.odrive.axis1.current_state

    def scale_setpoint(self, angle, gamma):
        front_rad = angle+gamma
        back_rad = angle-gamma
        count_front_offset, count_back_offset = ((math.pi/2)*self.front_cprad), ((math.pi/2)*self.back_cprad)
        count_front = (front_rad * self.front_cprad)- count_front_offset + self.offset_front
        count_back = (back_rad * self.back_cprad) + count_back_offset + self.offset_back
        #print("angle: ", angle, "gomma: ", gamma)
        #print("count_front: ", count_front, "count_back: ", count_back)
        front_n = (count_front - self.offset_front + ((math.pi/2)*self.front_cprad)) / self.front_cprad
        back_n =  (count_back - self.offset_back - ((math.pi/2)*self.back_cprad)) / self.back_cprad
        #print("front_deg_d: ", math.degrees(front_n), "bac_deg_d: ", math.degrees(back_n))
        
        return count_front, count_back

    def enable(self):
        if self.enabled and self.connected:
            self.odrive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            self.odrive.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    def disable(self):
       if self.enabled and self.connected:
           self.odrive.axis0.requested_state = AXIS_STATE_IDLE
           self.odrive.axis1.requested_state = AXIS_STATE_IDLE
           
    def calibrate_encoders(self):
       if self.enabled and self.connected:
           self.odrive.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
           self.odrive.axis1.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
           
    def reboot(self):
       if self.enabled and self.connected:
           self.odrive.reboot()

    def set_commands(self, angle, gamma):
        self.angle_setpoint = angle  
        self.gamma_setpoint = gamma 

    def run(self):
        if self.enabled and self.connected:
            if not math.isnan(self.angle_setpoint) and not math.isnan(self.gamma_setpoint):
                count_front, count_back = self.scale_setpoint(self.angle_setpoint, self.gamma_setpoint)

                self.odrive.axis0.controller.input_pos = count_front
                self.odrive.axis1.controller.input_pos = count_back
                
            else:
                pass

    def home(self):
        print('starting home')
        if self.enabled:
            self.offset_front = self.odrive.axis0.encoder.pos_estimate
            self.offset_back = self.odrive.axis1.encoder.pos_estimate
            print("back offset: ", self.offset_back, "front offset: ", self.offset_front)            
        print('ending home')

    def request_bus_voltage(self):
        if self.enabled and self.connected:
            return self.odrive.vbus_voltage
        else:
            return 0.0

   


class Drive_Base_Driver_Node(Node):
    def __init__(self):
        super().__init__('drive_base_driver_node')
        
        #Setting up subscriptions
        self.subscription = self.create_subscription(
            Twist,
            'dolly_cmd',
            self.dolly_cmd_callback,
            0)
        self.subscription  # prevent unused variable warning

        self.voltage_timer = self.create_timer(5, self.request_bus_voltage) #seconds
        self.bus_v_publisher = self.create_publisher(
            Float32,
            'battery_voltage',
            10)
        
        self.pose_publisher = self.create_publisher(
            Twist,
            'leg_pose',
            0)
        
        self.pos_timer = self.create_timer((1/200), self.request_pos)
        self.watch_dog_timer = self.create_timer(5, self.watch_dog)
        
        self.enable_dolly_srv = self.create_service(
            Trigger,
            'enable_dolly',
            self.enable_dolly_callback)
        
        self.disable_dolly_srv = self.create_service(
            Trigger,
            'disable_dolly',
            self.disable_dolly_callback)
        
        self.home_dolly_srv = self.create_service(
            Trigger,
            'home_dolly',
            self.home_dolly_callback)
        
        self.calibrate_dolly_srv = self.create_service(
            Trigger,
            'calibrate_encoders',
            self.calibrate_encoders_callback)
        
        self.reboot_dolly_srv = self.create_service(
            Trigger,
            'reboot_dolly',
            self.reboot_dolly_callback)
        
        #setting up dolly class
        with open("/home/autodollyv2/doggo_ws/src/config.json") as json_file: # open config.json
            config = json.load(json_file) # load configuration

        self.dolly = Dolly(config['dolly'])
        self.packet = copy.copy(packet)
        

    def dolly_cmd_callback(self, msg):
        self.packet = copy.copy(packet)
        self.packet['doggo']['right']['x'] = msg.linear.x
        self.packet['doggo']['right']['y'] = msg.linear.y
        self.packet['doggo']['left']['x'] = msg.angular.x
        self.packet['doggo']['left']['y'] = msg.angular.y
        
        self.dolly.set_commands(self.packet['doggo']) # every axis should have a method called set_commands() that sets the current setpoints for that axis
        self.dolly.run()
            
        #print(self.packet)
    
    def request_bus_voltage(self):
        voltage = self.dolly.modules['back'].request_bus_voltage()
        print("voltage_callback")
        msg = Float32()
        msg.data = voltage
        self.bus_v_publisher.publish(msg)
        
    def request_pos(self):
        self.dolly.request_pos()
        msg_out = Twist()
        if self.dolly.modules['back'].back_n is not None and self.dolly.modules['back'].front_n is not None:
            msg_out.linear = Vector3(x=0.0 , y=0.0, z=0.0)
            msg_out.angular = Vector3(x=self.dolly.modules['back'].back_n, y=self.dolly.modules['back'].front_n, z=0.0)
            self.pose_publisher.publish(msg_out)
    
   
    def update_pos_estimate(self):
        self.dolly.update_pos(data)
        
    def watch_dog(self):
        self.dolly.check_state()

    def calibrate_encoders_callback(self, request, response):
        self.dolly.calibrate_encoders()
        print("Send Dolly calibrate cmd")
        response.success = True
        response.message = "calibrating"
        return response
    
    def enable_dolly_callback(self, request, response):
        start_time = time.time()
        print("Send Dolly enable cmd")
        self.dolly.enable()
        done = False
        while not done:
            if not False in np.where(self.dolly.check_state() == 8, True, False):
                done = True
                response.success = True
                response.message = "Success"
            elif time.time() - start_time > 2:
                done = True
                print("Failed Enable timeout")
                response.success = False
                response.message = "Failed"
        return response
    
    def disable_dolly_callback(self, request, response):
        start_time = time.time()
        self.dolly.disable()
        print("Send Dolly disable cmd")
        done = False
        while not done:
            if not False in np.where(self.dolly.check_state() == 1, True, False):
                done = True
                response.success = True
                response.message = "Success"
            elif time.time() - start_time > 2:
                done = True
                print("Failed Enable timeout")
                response.success = False
                response.message = "Failed"
        return response
    
    def home_dolly_callback(self, request, response):
        self.dolly.home()
        print("Send Doggo home cmd")
        response.success = True
        response.message = "Homed"
        return response
    
    def reboot_dolly_callback(self, request, response):
        self.dolly.reboot()
        print("Send Doggo reboot cmd")
        response.success = True
        response.message = "rebooting"
        return response
    
def main(args=None):
    rclpy.init(args=args)

    drive_base_driver_node = Drive_Base_Driver_Node()

    rclpy.spin(drive_base_driver_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    drive_base_driver_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
