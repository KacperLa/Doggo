import math
import os
import json
import odrive
from odrive.enums import *
import asyncio
from odrive.utils import dump_errors as d_e


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

        # set points
        self.theta_setpoint = None
        self.omega_dot_setpoint = None
        self.velocity_setpoint = None

        # motion limits *implement these in the kinematics stuff as governers
        self.velocity_limit = None
        self.omega_dot_limit = None

        #  Create three Modules using the imported config file
        self.modules = {
            'back': Module('back', config['back'], self.enabled),
            'left': Module('left', config['left'], self.enabled),
            'right': Module('right', config['right'], self.enabled),
        }

    async def setup(self, settings):
        """
        Function:
        Initialize each module
        set tuning parameters
        home them concurrently.
        :param settings: tuning parameters
        """
        print('Starting Dolly Setup')
        if self.enabled:
            print(self.modules)
            for key, module in self.modules.items():
                await module.connect()
                print('module found')

            #self.set_settings(settings)

            tasks = []
            for key, module in self.modules.items():
                tasks.append(module.start())
            await asyncio.gather(*tasks)

            self.enable()

            print('Done with Dolly Setup')

    def dump_errors(self):
        """
        Overwrites inherited function from axis object
        Call dump_errors for each module(axis)
        """
        for key, module in self.modules.items():
            module.dump_errors()

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
        self.theta_setpoint = commands['theta']
        self.omega_dot_setpoint = commands['omega_dot']
        self.velocity_setpoint = commands['velocity']

        #elf.run()

    def run(self):
        """
        Overwrite inherited function
        Calculate angle and velocity for each module
        Pass these values on to each module
        call run for each module(axis)
        """
        angle, v = self.inverse_kinematics(self.theta_setpoint, self.omega_dot_setpoint, self.velocity_setpoint)
        angle_1, angle_2, angle_3 = angle
        v_1, v_2, v_3 = v

        if self.enabled:
            self.modules['left'].set_commands(angle_1, v_1)
            self.modules['right'].set_commands(angle_2, v_2)
            self.modules['back'].set_commands(angle_3, v_3)

            for key, module in self.modules.items():
                module.run()

    def calibrate(self):
        if self.enabled:
            for key, module in self.modules.items():
                module.calibrate()

    def e_stop(self):
        """
        Overwrites inherited function from axis object
        Run .e_stop function for each Module(axis)
        """
        for key, module in self.modules.items():
            module.e_stop()

    def inverse_kinematics(self, theta, w, v):
        """
        This kinematic model is a copy of the kinematic model from the PC_UI
        any changes made here should be replicated in the PC_UI kinematic
        model to ensure that the dolly behaves as expected
        :param theta: direction of dolly's velocity vector in dolly coordinate system
        :param w: magnitude of dolly's rotational velocity
        :param v: magnitude of dolly's velocity
        :return: angle and velocity for each module
        """
        w = w * 1
        v_x = v * math.cos(theta)
        v_y = v * math.sin(theta)
        w_x_1 = w * math.cos(-math.pi / 6.0)
        w_y_1 = w * math.sin(-math.pi / 6.0)
        w_x_2 = -w * math.cos(math.pi / 6.0)
        w_y_2 = -w * math.sin(math.pi / 6.0)
        w_x_3 = w * math.cos(math.pi / 2.0)
        w_y_3 = w * math.sin(math.pi / 2.0)
        x_1 = v_x + w_x_1
        y_1 = v_y + w_y_1
        x_2 = v_x + w_x_2
        y_2 = v_y + w_y_2
        x_3 = v_x + w_x_3
        y_3 = v_y + w_y_3

        angle_1 = math.atan2(y_1, x_1)
        if angle_1 < 0.0:
            angle_1 = angle_1 + 2.0 * math.pi

        angle_2 = math.atan2(y_2, x_2)
        if angle_2 < 0:
            angle_2 = angle_2 + 2.0 * math.pi

        angle_3 = math.atan2(y_3, x_3)
        if angle_3 < 0:
            angle_3 = angle_3 + 2.0 * math.pi

        angle_1 = angle_1  # top left
        angle_2 = angle_2  # top right
        angle_3 = angle_3  # bottom

        v_1 = math.sqrt(pow(x_1, 2) + pow(y_1, 2))
        v_2 = math.sqrt(pow(x_2, 2) + pow(y_2, 2))
        v_3 = math.sqrt(pow(x_3, 2) + pow(y_3, 2))

        return [[angle_1, angle_2, angle_3], [v_1, v_2, v_3]]

    def set_settings(self, settings_obj):
        # print(f'settings object: {settings_obj}')

        temp = settings_obj.get('velocity_limit')
        if temp:
            self.velocity_limit = temp

        temp = settings_obj.get('omega_dot_limit')
        if temp:
            self.omega_dot_limit = temp

        for key, module in self.modules.items():
            module.set_settings(settings_obj.get('module'))

    def get_settings(self):
        response = {
            'velocity_limit': self.velocity_limit,
            'omega_dot_limit': self.omega_dot_limit,
            'module': self.modules['back'].get_settings(),
        }
        return response


class Pedestal(Axis):
    """
    Pedestal Class: inheres its structure from the Axis Object
    Responsible for handling the function of the vertical axis
    """
    def __init__(self, config):
        self.enabled = config['enabled']

        # odrive
        self.odrive_serial_number = config['odrive_serial_number']
        self.odrive = None

        # setpoints
        self.z_dot_setpoint = 0
        self.velocity_limit = None
        self.current_limit = None

    async def setup(self, settings):
        if self.enabled:
            self.odrive = odrive.find_any(serial_number=str(self.odrive_serial_number))
            self.set_settings(settings)
            # put all axes in idle state
            self.disable()

    def enable(self):
        if self.enabled:
            self.odrive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            self.odrive.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    def disable(self):
        if self.enabled:
            self.odrive.axis0.requested_state = AXIS_STATE_IDLE
            self.odrive.axis1.requested_state = AXIS_STATE_IDLE

    def set_settings(self, settings_obj):
        if self.enabled:
            temp = settings_obj.get('z').get('velocity_limit')
            if temp:
                self.velocity_limit = temp
                self.odrive.axis0.controller.config.vel_limit = temp

            temp = settings_obj.get('z').get('current_limit')
            if temp:
                self.current_limit = temp
                self.odrive.axis0.motor.config.current_lim = temp

            temp = settings_obj.get('z').get('k_v')
            if temp:
                self.odrive.axis0.controller.config.vel_gain = temp

            temp = settings_obj.get('z').get('k_vi')
            if temp:
                self.odrive.axis0.controller.config.vel_integrator_gain = temp

    def get_settings(self):
        if self.enabled:
            response = {
                'z': {
                    'velocity_limit': self.odrive.axis0.controller.config.vel_limit,
                    'current_limit': self.odrive.axis0.motor.config.current_lim,
                    'k_v': self.odrive.axis0.controller.config.vel_gain,
                    'k_vi': self.odrive.axis0.controller.config.vel_integrator_gain,
                }
            }
            return response
        return None

    def set_commands(self, commands):
        self.z_dot_setpoint = commands['z_dot']

    def run(self):
        pass


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

        # ODrive
        self.odrive_serial_number = config['odrive_serial_number']
        self.odrive_name = name
        self.odrive = None

        # constants
        self.velocity_cpr = 90
        self.theta_cpr = 8192
        self.velocity_max_speed = 90

        # calibration variables
        self.hall_offset = config['hall_offset']
        self.theta_offset = config['theta_offset']
        self.boot_offset = None

        # set points
        self.velocity_setpoint = None
        self.theta_setpoint = None

        # Variable used in determining the direction the velocity wheel spins
        # Bool value set in scale_theta_function
        self.reverse = 0

    async def connect(self):
        print('starting module')
        print(self.enabled)
        if self.enabled:
            self.odrive = odrive.find_any(serial_number=str(self.odrive_serial_number))
            print(f'{self.odrive_name} ODrive found')
            # put all axes in idle state
            self.disable()

    async def start(self):
        print('starting module')
        await self.home()

    def dump_errors(self):
        """
        Calling odrive.dump_errors function on a self.odrive
        """
        d_e(self.odrive, True)

    def e_stop(self):
        """
        Trying to set each axis to IDLE
        """
        if self.enabled:
            try:
                self.odrive.axis0.controller.vel_setpoint = 0
            except:
                print("Failed to stop velocity for module:", self.module_name)
            try:
                self.odrive.axis0.requested_state = AXIS_STATE_IDLE
            except:
                print("Failed to shutdown velocity motor for module:", self.module_name)
            try:
                self.odrive.axis1.requested_state = AXIS_STATE_IDLE
            except:
                print("Failed ot shutdown theta motor for module:", self.module_name)
            print("ODRIVES SHUTDOWN")

    def scale_theta(self, cmd):
        """
        Function responsible for finding the closest path to command theta
        :param cmd: desired theta position
        :return cmd: augmented theta position relative to current theta position
        """
        measured = (2.0 * math.pi) * ((self.odrive.axis1.encoder.pos_estimate - self.offset) / (self.theta_cpr * 4.5))
        mod = 2 * math.pi
        measured_mod = math.fmod(measured, mod) + mod
        offsets = [
            math.fmod(math.fmod(cmd - measured_mod, mod) + mod, mod),
            math.fmod(math.fmod(cmd - measured_mod + mod / 2.0, mod) + mod, mod),
            -1.0 * math.fmod(math.fmod(measured_mod - cmd, mod) + mod, mod),
            -1.0 * math.fmod(math.fmod(measured_mod + mod / 2.0 - cmd, mod) + mod, mod),
        ]

        min_val = 2 * math.pi
        min_ind = 0
        for ind, val in enumerate(offsets):
            if abs(val) < abs(min_val):
                min_val = val
                min_ind = ind

        if min_ind == 1 or min_ind == 3:
            self.reverse = True
        else:
            self.reverse = False

        cmd = self.offset + (((measured + min_val) / (2 * math.pi)) * self.theta_cpr * 4.5)

        return cmd

    def enable(self):
        if self.enabled:
            self.odrive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            self.odrive.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    def disable(self):
        if self.enabled:
            self.odrive.axis0.requested_state = AXIS_STATE_IDLE
            self.odrive.axis1.requested_state = AXIS_STATE_IDLE

    def scale_velocity(self, cmd):
        if self.reverse:
            return cmd * -1 * self.velocity_max_speed
        else:
            return cmd * self.velocity_max_speed

    def set_commands(self, theta, velocity):
        """
        This function is responsible for receiving theta and velocity commands
        from an outside source
        :param theta: requested theta position in radian's
        :param velocity:  requested theta setpoint scaled form 0-1
        :return:
        self.theta_setpoint: Requested theta position converted to encoder pulses
        self.velocity_setpoint: Requested velocity set point converted to pulses/sec
        """
        self.theta_setpoint = theta  # scaled in pulses of the encoder
        self.velocity_setpoint = velocity  # scaled to pulses/sec

    def run(self):
        if self.enabled:
            if self.velocity_setpoint is not None:
                v = self.scale_velocity(self.velocity_setpoint)
                self.odrive.axis0.controller.vel_setpoint = v
            else:
                self.odrive.axis0.controller.vel_setpoint = 0

            if self.theta_setpoint is not None:
                theta = self.scale_theta(self.theta_setpoint)
                self.odrive.axis1.controller.pos_setpoint = theta

    def find_hall_offset2(self):
        if self.enabled:
            print(self.boot_offset)
            self.odrive.axis1.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
            self.odrive.axis1.controller.vel_setpoint = self.theta_cpr * 0.20  # counts per second
            time.sleep(.5)
            last_hall = self.odrive.get_adc_voltage(5)
            while True:
                current_hall = self.odrive.get_adc_voltage(5)
                if last_hall >= 2.0 and current_hall < 2.0:
                    falling_edge = self.odrive.axis1.encoder.pos_estimate
                    break
                last_hall = current_hall

            while True:
                current_hall = self.odrive.get_adc_voltage(5)
                if last_hall < 2.0 and current_hall >= 2.0:
                    rising_edge = self.odrive.axis1.encoder.pos_estimate
                    break
                last_hall = current_hall

            magnet_center = (rising_edge + falling_edge) / 2.0
            hall_offset = round(magnet_center - self.boot_offset)

            self.odrive.axis1.controller.vel_setpoint = 0.0  # counts per second
            time.sleep(.5)
            self.odrive.axis1.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
            self.odrive.axis1.controller.pos_setpoint = magnet_center  # counts per second

            print('magnet_center : ' + str(magnet_center))
            print('hall_offset : ' + str(hall_offset))

    async def home(self):
        print('starting home')
        if self.enabled:
            self.boot_offset = self.odrive.axis1.encoder.pos_estimate
            self.enable()
            await asyncio.sleep(0.1)

            self.odrive.axis1.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
            self.odrive.axis1.controller.vel_setpoint = self.theta_cpr * 0.40  # counts per second

            await asyncio.sleep(0.1)

            last_hall = self.odrive.get_adc_voltage(5)
            while True:
                current_hall = self.odrive.get_adc_voltage(5)
                if last_hall >= 2.0 and current_hall < 2.0:
                    falling_edge = self.odrive.axis1.encoder.pos_estimate
                    break
                last_hall = current_hall
                await asyncio.sleep(0.001)  # give a little time to run other tasks

            while True:
                current_hall = self.odrive.get_adc_voltage(5)
                if last_hall < 2.0 and current_hall >= 2.0:
                    rising_edge = self.odrive.axis1.encoder.pos_estimate
                    break
                last_hall = current_hall
                await asyncio.sleep(0.001)  # give a little time to run other tasks

            magnet_center = (rising_edge + falling_edge) / 2.0
            self.odrive.axis1.controller.vel_setpoint = 0.0  # counts per second
            start_pos = magnet_center + ((self.theta_offset / 360.0) * (self.theta_cpr * 4.5))
            self.odrive.axis1.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
            self.odrive.axis1.controller.pos_setpoint = start_pos  # counts
            print("self.boot_offset : " + str(self.boot_offset) + " self.theta_offset : " + str(self.theta_offset)
                  + "start_pos : " + str(start_pos) + " magnet_center : " + str(magnet_center))
            self.offset = start_pos
            await asyncio.sleep(3.0)

            self.disable()
        print('ending home')

    def get_vin(self):
        """
        Pull bus voltage from odrive
        :return: return bus voltage
        """
        return self.odrive.vbus_voltage

    def set_settings(self, settings_obj):
        # reminder axis[0] is velocity
        # reminder axis[1] is theta

        if self.enabled:
            temp = settings_obj.get('theta').get('velocity_limit')
            if temp:
                self.odrive.axis1.controller.config.vel_limit = temp

            temp = settings_obj.get('theta').get('current_limit')
            if temp:
                self.odrive.axis1.motor.config.current_lim = temp

            temp = settings_obj.get('theta').get('k_p')
            if temp:
                self.odrive.axis1.controller.config.pos_gain = temp

            temp = settings_obj.get('theta').get('k_v')
            if temp:
                self.odrive.axis1.controller.config.vel_gain = temp

            temp = settings_obj.get('theta').get('k_vi')
            if temp:
                self.odrive.axis1.controller.config.vel_integrator_gain = temp

            temp = settings_obj.get('velocity').get('velocity_limit')
            if temp:
                self.odrive.axis0.controller.config.vel_limit = temp

            temp = settings_obj.get('velocity').get('current_limit')
            if temp:
                self.odrive.axis0.motor.config.current_lim = temp

            temp = settings_obj.get('velocity').get('k_p')
            if temp:
                self.odrive.axis0.controller.config.pos_gain = temp

            temp = settings_obj.get('velocity').get('k_v')
            if temp:
                self.odrive.axis0.controller.config.vel_gain = temp

            temp = settings_obj.get('velocity').get('k_vi')
            if temp:
                self.odrive.axis0.controller.config.vel_integrator_gain = temp

    def get_settings(self):
        if self.enabled:
            response = {
                'theta': {
                    'velocity_limit': self.odrive.axis1.controller.config.vel_limit,
                    'current_limit': self.odrive.axis1.motor.config.current_lim,
                    'k_p': self.odrive.axis1.controller.config.pos_gain,
                    'k_v': self.odrive.axis1.controller.config.vel_gain,
                    'k_vi': self.odrive.axis1.controller.config.vel_integrator_gain,
                },
                'velocity': {
                    'velocity_limit': self.odrive.axis0.controller.config.vel_limit,
                    'current_limit': self.odrive.axis0.motor.config.current_lim,
                    'k_p': self.odrive.axis0.controller.config.pos_gain,
                    'k_v': self.odrive.axis0.controller.config.vel_gain,
                    'k_vi': self.odrive.axis0.controller.config.vel_integrator_gain,
                }
            }
        else:
            response = {}
        return response
