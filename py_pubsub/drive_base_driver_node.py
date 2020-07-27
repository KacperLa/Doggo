import json
import math
import struct
import copy
import numpy as np
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float32
from std_srvs.srv import Trigger

import can
can.rc['interface'] = 'socketcan'
can.rc['channel'] = 'can0'
can.rc['bitrate'] = 1000000
from can import Bus
bus = Bus()
bus.receive_own_messages = False

class FilteredCanReader(can.BufferedReader):
    def __init__(self):
        can.BufferedReader.__init__(self)
        self.func_map = {}
        self.domain = None
        
    def register_callback(self, address):
        def wrapper(function):
            self.func_map[str(int(address))] = function
            return function
        return wrapper

    def on_message_received(self, data=None):
        bin_id = bin(data.arbitration_id)
        if len(bin_id) > 7:
            id = str(int(bin_id[7:],2))
            func = self.func_map.get(id, None)
            if func is None:
                print("No function registered against")
                print(data)
            else:
                return func(self.domain, data)
        else:
            print("Can Address to short")
            print(data)
        
    
fbr = FilteredCanReader()
notifier = can.Notifier(bus, [fbr])

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
    def __init__(self, bus, config):
        """
        Initialization of the Dolly Class:
        define class variables:
        enable: what is this !!!!!!
        set points: variables set by the set commands function used in the run function
        :param
        config: JSON file with parameters relevant to the initialized axes
        """
        self.bus = bus
        
        self.enabled = config['enabled']

        # set points
        self.x_setpoint = None
        self.y_setpoint = None


        # motion limits *implement these in the kinematics stuff as governers
        #self.theta_limit = None
        #self.gamma_limit = None

        #  Create three Modules using the imported config file
        self.modules = {
            'back': Module(self.bus, 'back', config['back'], self.enabled),
            'left': Module(self.bus, 'left', config['left'], self.enabled),
            #'right': Module(self.bus, 'right', config['right'], self.enabled),
        }
        self.odrive_ids = [self.modules['back'].odrive_id_front, self.modules['back'].odrive_id_back, self.modules['left'].odrive_id_front, self.modules['left'].odrive_id_back]

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
    def __init__(self, bus, name, config, enabled):
        """
        Initialise ODrive using given config JSON object
        :param config: Properties specific to given module
        :param enabled:
        """
        self.bus = bus
        self.enabled = enabled
        self.state = [0, 0]

        # ODrive
        self.odrive_name = name
        self.odrive = None
        print(config)
        self.odrive_id_front = int(config['node_id_front'],16)
        self.odrive_id_back = int(config['node_id_back'],16)

        # constants
        self.front_cprad = (2000*4.5)/(-math.pi*2)
        self.back_cprad = (2000*4.5)/(-math.pi*2)


        # calibration variables
        self.back_offset = config['back_offset']
        self.front_offset = config['front_offset']
        self.boot_offset_front = None
        self.boot_offset_back = None
        self.offset_front = 0
        self.offset_back = 0
        self.pos_estimate_front = 0
        self.pos_estimate_back = 0
        # set points
        self.back_setpoint = None
        self.front_setpoint = None
        self.back_n = None
        self.front_n = None
        
    def request_pos(self):
        msg1 = can.Message(arbitration_id=self.odrive_id_front + 0x009, data=[],is_remote_frame=True, is_extended_id=False)
        msg2 = can.Message(arbitration_id=self.odrive_id_back + 0x009, data=[],is_remote_frame=True, is_extended_id=False)
        self.bus.send(msg1)
        self.bus.send(msg2)

    def update_pos(self, data):
        pos_estimate = struct.unpack('f', data.data[0:4])[0]
        if data.arbitration_id == self.odrive_id_front + 0x009 and data.dlc == 8:
            self.pos_estimate_front = pos_estimate
        elif data.arbitration_id == self.odrive_id_back + 0x009 and data.dlc == 8:
            self.pos_estimate_back = pos_estimate
        #print("front: ", self.pos_estimate_front, "back: ", self.pos_estimate_back)
        self.front_n = (self.pos_estimate_front - self.front_offset + ((math.pi/2)*self.front_cprad)) / self.front_cprad
        self.back_n =  (self.pos_estimate_back - self.back_offset - ((math.pi/2)*self.back_cprad)) / self.back_cprad
        #print("front_deg_real: ", math.degrees(front_n), "bac_deg_real: ", math.degrees(back_n))
        #print("encoder pos: ", struct.unpack('f', data.data[0:4]))
        #print("encoder vel: ", struct.unpack('f', data.data[4:8])) 

    def update_state(self, data):
        data_unpacked = struct.unpack('II', data.data)[1]
        if data.arbitration_id == self.odrive_id_front + 0x001 and data.dlc == 8:
            self.state[0] = data_unpacked
        elif data.arbitration_id == self.odrive_id_back + 0x001 and data.dlc == 8:
            self.state[1] = data_unpacked

    def start(self):
        print('starting module')
        #await self.home()

    def scale_setpoint(self, angle, gamma):
        front_rad = angle+gamma
        back_rad = angle-gamma
        count_front_offset, count_back_offset = ((math.pi/2)*self.front_cprad), ((math.pi/2)*self.back_cprad)
        count_front = (front_rad * self.front_cprad)- count_front_offset + self.front_offset
        count_back = (back_rad * self.back_cprad) + count_back_offset + self.back_offset
        #print("angle: ", angle, "gomma: ", gamma)
        #print("count_front: ", count_front, "count_back: ", count_back)
        front_n = (count_front - self.front_offset + ((math.pi/2)*self.front_cprad)) / self.front_cprad
        back_n =  (count_back - self.back_offset - ((math.pi/2)*self.back_cprad)) / self.back_cprad
        #print("front_deg_d: ", math.degrees(front_n), "bac_deg_d: ", math.degrees(back_n))
        
        return count_front, count_back

    def enable(self):
        if self.enabled:
            msg_1 = can.Message(arbitration_id=self.odrive_id_front + 0x007, data=struct.pack('I', 8),is_remote_frame=False, is_extended_id=False)
            msg_2 = can.Message(arbitration_id=self.odrive_id_back + 0x007, data=struct.pack('I', 8),is_remote_frame=False, is_extended_id=False)
            self.bus.send(msg_1)
            self.bus.send(msg_2)

    def disable(self):
       if self.enabled:
           msg_1 = can.Message(arbitration_id=self.odrive_id_front + 0x007, data=struct.pack('I', 1),is_remote_frame=False, is_extended_id=False)
           msg_2 = can.Message(arbitration_id=self.odrive_id_back + 0x007, data=struct.pack('I', 1),is_remote_frame=False, is_extended_id=False)
           self.bus.send(msg_1)
           self.bus.send(msg_2)
           
    def calibrate_encoders(self):
       if self.enabled:
           msg_1 = can.Message(arbitration_id=self.odrive_id_front + 0x007, data=struct.pack('I', 7),is_remote_frame=False, is_extended_id=False)
           msg_2 = can.Message(arbitration_id=self.odrive_id_back + 0x007, data=struct.pack('I', 7),is_remote_frame=False, is_extended_id=False)
           self.bus.send(msg_1)
           self.bus.send(msg_2)
           
    def reboot(self):
       if self.enabled:
           msg_1 = can.Message(arbitration_id=self.odrive_id_front + 0x016, is_remote_frame=False, is_extended_id=False)
           self.bus.send(msg_1)
           print("rebooting")

    def set_commands(self, angle, gamma):
        self.angle_setpoint = angle  # scaled mm
        self.gamma_setpoint = gamma # scaled mm

    def run(self):
        if self.enabled:
            if not math.isnan(self.angle_setpoint) and not math.isnan(self.gamma_setpoint):
                count_front, count_back = self.scale_setpoint(self.angle_setpoint, self.gamma_setpoint)

                msg_front = can.Message(arbitration_id=self.odrive_id_front + 0x00c, data=struct.pack('ihh', int(count_front), 0, 0),is_remote_frame=False, is_extended_id=False)
                msg_back = can.Message(arbitration_id=self.odrive_id_back + 0x00c, data=struct.pack('ihh', int(count_back), 0, 0),is_remote_frame=False, is_extended_id=False)
                self.bus.send(msg_front)
                self.bus.send(msg_back)
            else:
                pass

    def home(self):
        print('starting home')
        if self.enabled:
            self.front_offset = self.pos_estimate_front
            self.back_offset = self.pos_estimate_back
            print("back offset: ", self.back_offset, "front offset: ", self.front_offset)            
        print('ending home')

    def request_bus_voltage(self):
        """
        Send bus voltage request to odrive
        :return: return bus voltage
        """
        msg = can.Message(arbitration_id=self.odrive_id_front + 0x017, data=[],is_remote_frame=True, is_extended_id=False)
        bus.send(msg)

   


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

        self.dolly = Dolly(bus, config['dolly'])
        self.packet = copy.copy(packet)
        fbr.domain = self
        

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
        self.dolly.modules['back'].request_bus_voltage()
        
    def request_pos(self):
        self.dolly.request_pos()
        msg_out = Twist()
        if self.dolly.modules['back'].back_n is not None and self.dolly.modules['back'].front_n is not None:
            msg_out.linear = Vector3(x=0.0 , y=0.0, z=0.0)
            msg_out.angular = Vector3(x=self.dolly.modules['back'].back_n, y=self.dolly.modules['back'].front_n, z=0.0)
            self.pose_publisher.publish(msg_out)
    
    @fbr.register_callback(0x017)
    def publish_bus_voltage(self, data):
        print("voltage_callback")
        msg = Float32()
        msg.data = struct.unpack('2f', data.data)[0]
        self.bus_v_publisher.publish(msg)
    
    @fbr.register_callback(0x009)
    def update_pos_estimate(self, data):
        self.dolly.update_pos(data)
        
    @fbr.register_callback(0x001)
    def watch_dog(self, data):
        self.dolly.update_state(data)

        
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
