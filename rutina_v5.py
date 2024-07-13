#!/usr/bin/env python3
#
# Author: Jorge Ramirez <jorge.ramirezc@pucp.pe> 

"""
# Source:
# xArm-Python-SDK: https://github.com/xArm-Developer/xArm-Python-SDK
#   1. git clone git@github.com:xArm-Developer/xArm-Python-SDK.git
#   2. cd xArm-Python-SDK
#   3. python setup.py install
"""
import sys
import math
import time
import queue
import datetime
import random
import traceback
import threading
from xarm import version
from xarm.wrapper import XArmAPI


class RobotMain(object):
    """Robot Main Class"""
    def __init__(self, robot, **kwargs):
        self.alive = True
        self._arm = robot
        self._tcp_speed = 100
        self._tcp_acc = 2000
        self._angle_speed = 20
        self._angle_acc = 500
        self._vars = {'timer': 0, 'out_bucle': 0}
        self._funcs = {
            # Capacitive Sensors are used to validate the position of the robot gripper 
        
            "Wait capacitive sensor nutribullet - CI0": self.function_1, 
            "Wait capacitive sensor ice dispenser - CI5": self.function_2,

            #These functions have been disabled until the capacitive sensors of the ingredient dispenser subsystem are replaced.
            #"Wait capacitive sensor ingredient dispenser 1 - CI1": self.function_3,
            #"Wait capacitive sensor ingredient dispenser 2 - CI2": self.function_4,
            #"Wait capacitive sensor ingredient dispenser 3 - CI3": self.function_5,
            #"Wait capacitive sensor ingredient dispenser 4 - CI4": self.function_6,
        }
        self._robot_init()

    # Robot init
    def _robot_init(self):
        self._arm.clean_warn()
        self._arm.clean_error()
        self._arm.motion_enable(True)
        self._arm.set_mode(0)
        self._arm.set_state(0)
        time.sleep(1)
        self._arm.register_error_warn_changed_callback(self._error_warn_changed_callback)
        self._arm.register_state_changed_callback(self._state_changed_callback)
        if hasattr(self._arm, 'register_count_changed_callback'):
            self._arm.register_count_changed_callback(self._count_changed_callback)

    # Register error/warn changed callback
    def _error_warn_changed_callback(self, data):
        if data and data['error_code'] != 0:
            self.alive = False
            self.pprint('err={}, quit'.format(data['error_code']))
            self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)

    # Register state changed callback
    def _state_changed_callback(self, data):
        if data and data['state'] == 4:
            self.alive = False
            self.pprint('state=4, quit')
            self._arm.release_state_changed_callback(self._state_changed_callback)

    # Register count changed callback
    def _count_changed_callback(self, data):
        if self.is_alive:
            self.pprint('counter val: {}'.format(data['count']))

    def _check_code(self, code, label):
        if not self.is_alive or code != 0:
            self.alive = False
            ret1 = self._arm.get_state()
            ret2 = self._arm.get_err_warn_code()
            self.pprint('{}, code={}, connected={}, state={}, error={}, ret1={}. ret2={}'.format(label, code, self._arm.connected, self._arm.state, self._arm.error_code, ret1, ret2))
        return self.is_alive

    @staticmethod
    def pprint(*args, **kwargs):
        try:
            stack_tuple = traceback.extract_stack(limit=2)[0]
            print('[{}][{}] {}'.format(time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())), stack_tuple[1], ' '.join(map(str, args))))
        except:
            print(*args, **kwargs)

    @property
    def arm(self):
        return self._arm

    @property
    def VARS(self):
        return self._vars

    @property
    def FUNCS(self):
        return self._funcs

    @property
    def is_alive(self):
        if self.alive and self._arm.connected and self._arm.error_code == 0:
            if self._arm.state == 5:
                cnt = 0
                while self._arm.state == 5 and cnt < 5:
                    cnt += 1
                    time.sleep(0.1)
            return self._arm.state < 4
        else:
            return False

    def function_1(self):
        """
        Wait capacitive sensor is enabled
        """
        # Wait capacitive sensor
        self._vars['out_bucle'] = 0
        while self.is_alive and self._arm.get_cgpio_digital(0)[1] == 1:
            self._vars['timer'] = datetime.datetime.now()
            while self.is_alive and self._arm.get_cgpio_digital(0)[1] == 0:
                if (datetime.datetime.now() - self._vars.get('timer', 0)) >= datetime.timedelta(seconds=5):
                    self._vars['out_bucle'] = 1
                if self._vars.get('out_bucle', 0) == 1:
                    break
            if self._vars.get('out_bucle', 0) == 1:
                break

    def function_2(self):
        """
        Wait capacitive sensor is enabled
        """
        # Wait capacitive sensor
        self._vars['out_bucle'] = 0
        while self.is_alive and self._arm.get_cgpio_digital(5)[1] == 1:
            self._vars['timer'] = datetime.datetime.now()
            while self.is_alive and self._arm.get_cgpio_digital(5)[1] == 0:
                if (datetime.datetime.now() - self._vars.get('timer', 0)) >= datetime.timedelta(seconds=5):
                    self._vars['out_bucle'] = 1
                if self._vars.get('out_bucle', 0) == 1:
                    break
            if self._vars.get('out_bucle', 0) == 1:
                break

    # Robot Main Run
    def run(self):
        try:
            # Linear Motion Settings
            self._tcp_speed = 300
            self._tcp_acc = 200
            self._angle_speed = 40
            self._angle_acc = 382
            # Initial Postition
            code = self._arm.set_position(*[180.0, 170.0, 115.0, 180.0, 0.0, 90.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            # Open gripper lid
            code = self._arm.set_tgpio_digital(0, 0, delay_sec=0)
            if not self._check_code(code, 'set_tgpio_digital'):
                return
            code = self._arm.set_tgpio_digital(1, 1, delay_sec=0)
            if not self._check_code(code, 'set_tgpio_digital'):
                return
            # Start movement
            code = self._arm.set_position(*[180.0, 62.0, 115.0, 180.0, 0.0, 90.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[180.0, 62.0, 185.0, 180.0, 0.0, 90.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[180.0, 62.0, 185.0, 180.0, 0.0, 0.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[180.0, 62.0, 208.0, 180.0, 0.0, 0.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            # Ingredient 1: White egg
            code = self._arm.set_position(*[195.0, 26.0, 208.0, 180.0, 0.0, -40.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_cgpio_digital(1, 1, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                return
            code = self._arm.set_pause_time(0.75)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.set_cgpio_digital(1, 0, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                return
            # Ingredient 2: Gum syrup
            code = self._arm.set_position(*[180.0, 62.0, 208.0, 180.0, 0.0, 0.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[426.0, 93.0, 205.0, 180.0, 0.0, -70.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_cgpio_digital(2, 1, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                return
            code = self._arm.set_pause_time(4)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.set_cgpio_digital(2, 0, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                return
            code = self._arm.set_position(*[270.0, 60.0, 208.0, 180.0, 0.0, 0.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            # Ingredient 3: Pisco
            code = self._arm.set_position(*[385.0, 97.0, 205.0, 180.0, 0.0, -18.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_cgpio_digital(3, 1, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                return
            code = self._arm.set_pause_time(1.6)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.set_cgpio_digital(3, 0, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                return
            code = self._arm.set_position(*[270.0, 60.0, 208.0, 180.0, 0.0, 0.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            # Ingredient 4: Lemon
            code = self._arm.set_position(*[324.0, 64.0, 200.0, 180.0, 0.0, 33.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_cgpio_digital(4, 1, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                return
            code = self._arm.set_pause_time(0.8)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.set_cgpio_digital(4, 0, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                return
            code = self._arm.set_position(*[200.0, 60.0, 208.0, 180.0, 0.0, 0.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[180.0, 170.0, 115.0, 180.0, 0.0, 90.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            # Ice dispenser
            code = self._arm.set_position(*[180.0, 260.0, 115.0, 180.0, 0.0, 90.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[180.0, 260.0, 115.0, 180.0, 0.0, 66.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[100.0, 366.0, 122.0, 180.0, 0.0, 27.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[170.0, 366.0, 122.0, 180.0, 0.0, 27.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            # Get Ice
            code = self._arm.set_position(*[170.0, 366.0, 122.0, 180.0, 0.0, 27.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            self.function_2()
            if not self.is_alive:
                return
            code = self._arm.set_cgpio_digital(5, 1, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                return
            code = self._arm.set_pause_time(2)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.set_cgpio_digital(5, 0, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                return
            code = self._arm.set_pause_time(3)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.set_cgpio_digital(5, 1, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                return
            code = self._arm.set_pause_time(1)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.set_cgpio_digital(5, 0, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                return
            code = self._arm.set_pause_time(5)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.set_position(*[100.0, 366.0, 122.0, 180.0, 0.0, 27.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[100.0, 255.0, 122.0, 180.0, 0.0, 30.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[100.0, 255.0, 122.0, 180.0, 0.0, 60.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            # Mixer
            code = self._arm.set_position(*[100.0, 255.0, 300.0, 180.0, 0.0, 60.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[-23.0, 260.0, 300.0, 180.0, 0.0, 79.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            # Check this position
            code = self._arm.set_position(*[-23.0, 260.0, 225.0, 180.0, 0.0, 79.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_pause_time(10)
            if not self._check_code(code, 'set_pause_time'):
                return
            # Start mixing
            # Close lid
            self.function_1()
            if not self.is_alive:
                return
            code = self._arm.set_tgpio_digital(0, 0, delay_sec=0)
            if not self._check_code(code, 'set_tgpio_digital'):
                return
            code = self._arm.set_tgpio_digital(1, 0, delay_sec=0)
            if not self._check_code(code, 'set_tgpio_digital'):
                return
            code = self._arm.set_pause_time(5)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.set_cgpio_digital(0, 1, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                return
            code = self._arm.set_pause_time(6)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.set_cgpio_digital(0, 0, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                return
            code = self._arm.set_pause_time(3)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.set_cgpio_digital(0, 1, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                return
            code = self._arm.set_pause_time(6)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.set_cgpio_digital(0, 0, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                return
            code = self._arm.set_pause_time(3)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.set_cgpio_digital(0, 1, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                return
            code = self._arm.set_pause_time(6)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.set_cgpio_digital(0, 0, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                return
            code = self._arm.set_position(*[-23.0, 260.0, 300.0, 180.0, 0.0, 79.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            # Start serving in 3 times
            code = self._arm.set_position(*[-23.0, 260.0, 490.0, 180.0, 0.0, 79.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            self._tcp_speed = 100
            self._tcp_acc = 100
            self._angle_speed = 10
            self._angle_acc = 200
            # Servido 1
            
            code = self._arm.set_pause_time(10)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.set_tgpio_digital(0, 0, delay_sec=0)
            if not self._check_code(code, 'set_tgpio_digital'):
                return
            code = self._arm.set_tgpio_digital(1, 1, delay_sec=0)
            if not self._check_code(code, 'set_tgpio_digital'):
                return
            code = self._arm.set_position(*[-23.0, 358.0, 510.0, 180.0, 0.0, 79.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[-14.0, 524.0, 583.0, 180.0, 0.0, 79.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[6.0, 521.0, 517.0, -157.0, 4.0, 86.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            
            code = self._arm.set_position(*[-60.0, 521.0, 592.0, -157.0, 4.0, 86.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[-1.0, 528.0, 515.0, -125.0, 2.5, 86.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[-16.0, 538.0, 482.0, -100.0, 1.6, 82.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_pause_time(0.7)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.set_position(*[-1.0, 528.0, 515.0, -125.0, 2.5, 86.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[-60.0, 521.0, 592.0, -157.0, 4.0, 86.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[-14.0, 524.0, 583.0, 180.0, 0.0, 79.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[-60.0, 414.0, 583.0, 180.0, 0.0, 79.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            # Circular mov to shake the mix
            self._tcp_speed = 600
            code = self._arm.move_circle([-200.0, 250.0, 470.0, 180.0, 0.0, 103.0], [-250.0, 200.0, 470.0, 180.0, 0.0, 103.0], float(720) / 360 * 100, speed=self._tcp_speed, mvacc=self._tcp_acc, wait=True)
            if not self._check_code(code, 'move_circle'):
                return
            self._tcp_speed = 100
            # Serving 2
            code = self._arm.set_position(*[-60.0, 414.0, 583.0, 180.0, 0.0, 79.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[-1.0, 528.0, 515.0, -125.0, 2.5, 86.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[-16.0, 538.0, 482.0, -100.0, 1.6, 82.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_pause_time(0.3)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.set_position(*[-1.0, 528.0, 515.0, -125.0, 2.5, 86.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[-60.0, 414.0, 583.0, 180.0, 0.0, 79.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            self._tcp_speed = 600
            code = self._arm.move_circle([-200.0, 250.0, 470.0, 180.0, 0.0, 103.0], [-250.0, 200.0, 470.0, 180.0, 0.0, 103.0], float(720) / 360 * 100, speed=self._tcp_speed, mvacc=self._tcp_acc, wait=True)
            if not self._check_code(code, 'move_circle'):
                return
            self._tcp_speed = 100
            # Serving 3
            code = self._arm.set_position(*[-60.0, 414.0, 583.0, 180.0, 0.0, 79.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[-1.0, 528.0, 515.0, -125.0, 2.5, 86.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[-16.0, 538.0, 482.0, -100.0, 1.6, 82.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_pause_time(0.3)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.set_position(*[-1.0, 528.0, 515.0, -125.0, 2.5, 86.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[-60.0, 414.0, 583.0, 180.0, 0.0, 79.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[-23.0, 260.0, 490.0, 180.0, 0.0, 79.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            # Return to home position
            code = self._arm.set_position(*[103.0, 260.0, 265.0, 180.0, 0.0, 79.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[180.0, 170.0, 115.0, 180.0, 0.0, 90.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            # Close lid
            code = self._arm.set_tgpio_digital(0, 0, delay_sec=0)
            if not self._check_code(code, 'set_tgpio_digital'):
                return
            code = self._arm.set_tgpio_digital(1, 0, delay_sec=0)
            if not self._check_code(code, 'set_tgpio_digital'):
                return
        except Exception as e:
            self.pprint('MainException: {}'.format(e))
        self.alive = False
        self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)
        self._arm.release_state_changed_callback(self._state_changed_callback)
        if hasattr(self._arm, 'release_count_changed_callback'):
            self._arm.release_count_changed_callback(self._count_changed_callback)


if __name__ == '__main__':
    RobotMain.pprint('xArm-Python-SDK Version:{}'.format(version.__version__))
    arm = XArmAPI('192.168.1.196', baud_checkset=False)
    robot_main = RobotMain(arm)
    robot_main.run()
