""" Filling Manager Module

  This module contains an implementation of Filling Manger class,
  that takes care about managing the process of cup detecting,
  cup filling and the end of filling.


  https://github.com/tomash1234/robot-arm-filler
"""

import time

from arm_controller import ArmDriver, ArmCommunicator

from detectors import CupDetector
from detectors import ArmEndDetector
from python.arm_dimensions import ArmDimensionsJson
from python.pos_finder import PosFinder


"""List of states """
""" Arm starts moving towards cup"""
STATE_MOVING = 1
""" Tip of arm is in the cup"""
STATE_READY_TO_FILL = 2
""" The pump is running"""
STATE_FILLING = 3
""" The pump has stopped and the tip of arm is moved up"""
STATE_STOPPING = 4
""" Arm is in initial position and waits"""
STATE_IDLE = 5


class FillingManager:

    """Filling manager class
    Simple implementation of the filling logic.
    There is a time threshold to start looking for the cup
    """

    def __init__(self, ip_address, port, dimension_file='config.json'):
        """Inits Filling manager with predefined values and IP address, port from arguments
        Args:
            ip_address: IP address of the board
            port:   port on which the board is listening
            dimension_file: path to json where the arm dimensions are defined
        """
        self.CUP_HEIGHT = 14
        self.FILLING_TIME = 5

        dim = ArmDimensionsJson(dimension_file)
        self.cup_detector = CupDetector()
        self.arm_detector = ArmEndDetector()
        self.driver = ArmDriver(dim)
        self.arm_com = ArmCommunicator(ip_address, port)

        self.filled = True
        self.was_valid = True
        self.angles = None
        self.filling_time = time.time()

        self.init_pose = (20, self.CUP_HEIGHT, 4)
        self.pos_finder = PosFinder(self.init_pose)
        self.state = STATE_MOVING

        self.rest_pose()

    def process_pic(self, img):
        """Process image from webcamera
        Args:
            img:    opencv image
        """
        self.cup_detector.scan_pic(img)
        self.arm_detector.process(img)

        if self.state == STATE_MOVING:
            if not self.cup_detector.is_tracked():
                return
            if not self.arm_detector.is_tracked():
                return
            self.moving_arm_for_filling()
        elif self.state == STATE_FILLING:
            self.filling()
        elif self.state == STATE_STOPPING:
            self.stopping()
        elif self.state == STATE_IDLE:
            self.rest_pose()
            if time.time() - self.filling_time > 5:
                self.state = STATE_MOVING
                self.pos_finder.reset(self.init_pose)

    def moving_arm_for_filling(self):
        """Method called every iteration in moving state"""
        marker = self.arm_detector.get_pos()
        cup = self.cup_detector.get_pos()

        new_point, found = self.pos_finder.iteration(cup, marker, self.was_valid)
        if new_point is not None:
            self.move(new_point)
            if found:
                self.start_filling()

    def rest_pose(self):
        """Set robot arm to initial resting pose"""
        self.move(self.init_pose)

    def start_filling(self):
        """Put the tip of arm into cup and starts pump"""
        angles = [self.angles[0], self.angles[1], self.angles[2] - 15]
        self.arm_com.send_angles(self.driver.convert_angles(angles))

        self.filling_time = time.time()
        self.state = STATE_FILLING

    def filling(self):
        """Method called every iteration in filling state"""
        if time.time() - self.filling_time > self.FILLING_TIME:
            self.filling_time = time.time()
            self.state = STATE_STOPPING

    def stopping(self):
        """Method called every iteration in stopping state"""
        dist = 15 - self.angles[2]
        duration = time.time() - self.filling_time
        if duration > 3:
            self.state = STATE_IDLE
            self.filling_time = time.time()
            return

        current = self.angles[2] + dist * min(1, duration)
        angles = [self.angles[0], self.angles[1], current]
        self.arm_com.send_angles(self.driver.convert_angles(angles))

    def move(self, pos):
        """Move the tip of roboarm into destination point
            Args:
                pos:    (x, y, z) destination pos
        """
        ret = self.driver.find_angles_with_threshold(pos, 0.5)
        self.was_valid = True if ret else False
        if ret is None:
            return
        angles = ret['angles']
        self.angles = angles
        self.arm_com.send_angles(self.driver.convert_angles(angles))
