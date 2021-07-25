import time

import numpy as np

from arm_controller import ArmDriver, ArmCommunicator

from cup_detector import CupDetector
from arm_end_detector import ArmEndDetector
from python.arm_dimensions import ArmDimensionsJson
from python.pos_finder import PosFinder

STATE_MOVING = 1
STATE_READY_TO_FILL = 2
STATE_FILLING = 3
STATE_STOPPING = 4
STATE_IDLE = 5


class FillingManager:

    def __init__(self):
        self.CUP_HEIGHT = 14
        self.FILLING_TIME = 5

        dim = ArmDimensionsJson('config.json')
        self.cup_detector = CupDetector()
        self.arm_detector = ArmEndDetector()
        self.driver = ArmDriver(dim)
        self.arm_com = ArmCommunicator('192.168.137.156', 5101)

        self.filled = True
        self.was_valid = True
        self.angles = None
        self.filling_time = time.time()

        self.init_pose = (20, self.CUP_HEIGHT, 4)
        self.pos_finder = PosFinder(self.init_pose)
        self.state = STATE_MOVING

        self.rest_pose()

    def process_pic(self, img):
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
        marker = self.arm_detector.get_pos()
        cup = self.cup_detector.get_pos()

        new_point, found = self.pos_finder.iteration(cup, marker, self.was_valid)
        if new_point is not None:
            self.move(new_point)
            if found:
                self.start_filling()

    def filling(self):
        if time.time() - self.filling_time > self.FILLING_TIME:
            self.filling_time = time.time()
            self.state = STATE_STOPPING

    def rest_pose(self):
        self.move(self.init_pose)

    def start_filling(self):
        angles = [self.angles[0], self.angles[1], self.angles[2] - 15]
        self.arm_com.send_angles(self.driver.convert_angles(angles))

        self.filling_time = time.time()
        self.state = STATE_FILLING

    def stopping(self):
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
        ret = self.driver.find_angles_with_threshold(pos, 0.5)
        self.was_valid = True if ret else False
        if ret is None:
            return None
        angles = ret['angles']
        self.angles = angles
        self.arm_com.send_angles(self.driver.convert_angles(angles))
