import numpy as np

from arm_controller import ArmDriver, ArmCommunicator

from cup_detector import CupDetector
from arm_end_detector import ArmEndDetector
from python.arm_dimensions import ArmDimensionsJson
from python.pos_finder import PosFinder


class FillingManager:

    def __init__(self):
        self.CUP_HEIGHT = 13
        self.init_pose = (20, self.CUP_HEIGHT, 4)
        dim = ArmDimensionsJson('config.json')
        self.cup_detector = CupDetector()
        self.arm_detector = ArmEndDetector()
        self.driver = ArmDriver(dim)
        self.arm_com = ArmCommunicator('192.168.137.237', 5101)
        self.filled = True
        self.was_valid = True

        self.pos_finder = PosFinder(self.init_pose)

        self.rest_pose()

    def process_pic(self, img):
        self.cup_detector.scan_pic(img)
        self.arm_detector.process(img)

        if not self.cup_detector.is_tracked():
            return

        if not self.arm_detector.is_tracked():
            self.rest_pose()
            return

        marker = self.arm_detector.get_pos()
        cup = self.cup_detector.get_pos()

        new_point = self.pos_finder.iteration(cup, marker, self.was_valid)
        if new_point is not None:
            self.move(new_point)


    def rest_pose(self):
        self.move(self.init_pose)
        pass

    def filling_pose(self):
        pass

    def move(self, pos):
        ret = self.driver.find_angles(pos)
        self.was_valid = True if ret else False
        if ret is None:
            return None
        angles = ret['angles']
        print(angles)
        self.arm_com.send_angles(self.driver.convert_angles(angles))
