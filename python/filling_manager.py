import numpy as np

from arm_controller import ArmDriver, ArmCommunicator

from cup_detector import CupDetector
from arm_end_detector import ArmEndDetector
from python.arm_dimensions import ArmDimensionsJson


class FillingManager:

    def __init__(self):
        dim = ArmDimensionsJson('config.json')
        self.CUP_HEIGHT = 12
        self.cup_detector = CupDetector()
        self.arm_detector = ArmEndDetector()
        self.driver = ArmDriver(dim)
        self.arm_com = ArmCommunicator('192.168.137.237', 5101)
        self.filled = True

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

        if self.is_in(marker, cup):
            print('OK')
            return

    def is_in(self, marker, cup):
        r = cup[2]
        marker = np.array([marker[:2]])
        cup = np.array([cup[:2]])
        if np.linalg.norm(marker - cup) < r * 0.75:
            return True
        return False

    def rest_pose(self):
        self.move((20, self.CUP_HEIGHT, 8))
        pass

    def filling_pose(self):
        pass

    def move(self, pos):
        ret = self.driver.find_angles(pos)
        angles = ret['angles']
        print(angles)
        self.arm_com.send_angles(self.driver.convert_angles(angles))
