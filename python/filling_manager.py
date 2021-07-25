import numpy as np

from arm_controller import ArmDriver, ArmCommunicator

from cup_detector import CupDetector
from arm_end_detector import ArmEndDetector
from python.arm_dimensions import ArmDimensionsJson
from python.pos_finder import PosFinder

STATE_MOVING = 1
STATE_READY_TO_FILL = 2
STATE_FILLING = 3
STATE_UP = 4
STATE_IDLE = 5


class FillingManager:

    def __init__(self):
        self.CUP_HEIGHT = 14
        dim = ArmDimensionsJson('config.json')
        self.cup_detector = CupDetector()
        self.arm_detector = ArmEndDetector()
        self.driver = ArmDriver(dim)
        self.arm_com = ArmCommunicator('192.168.137.243', 5101)

        self.filled = True
        self.was_valid = True
        self.angles = None

        self.init_pose = (20, self.CUP_HEIGHT, 4)
        self.pos_finder = PosFinder(self.init_pose)
        self.state = STATE_MOVING

        self.rest_pose()

    def process_pic(self, img):
        self.cup_detector.scan_pic(img)
        self.arm_detector.process(img)

        if not self.cup_detector.is_tracked():
            return

        if not self.arm_detector.is_tracked():
            return

        if self.state == STATE_MOVING:
            self.moving_arm_for_filling()

        if self.state == STATE_FILLING:
            # check if is ok
            pass

    def moving_arm_for_filling(self):
        marker = self.arm_detector.get_pos()
        cup = self.cup_detector.get_pos()

        new_point, found = self.pos_finder.iteration(cup, marker, self.was_valid)
        if new_point is not None:
            self.move(new_point)
            print(found)
            if found:
                self.filling_pose()
                self.state = STATE_FILLING

    def rest_pose(self):
        self.move(self.init_pose)
        pass

    def filling_pose(self):
        angles = [self.angles[0], self.angles[1], self.angles[2] - 15]
        self.arm_com.send_angles(self.driver.convert_angles(angles))
        pass

    def move(self, pos):
        ret = self.driver.find_angles_with_threshold(pos, 0.5)
        self.was_valid = True if ret else False
        if ret is None:
            return None
        angles = ret['angles']
        self.angles = angles
        self.arm_com.send_angles(self.driver.convert_angles(angles))
