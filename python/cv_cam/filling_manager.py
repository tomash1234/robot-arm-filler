import numpy as np

from cup_detector import CupDetector
from arm_end_detector import ArmEndDetector


class FillingManager:

    def __init__(self):
        self.cup_detector = CupDetector()
        self.arm_detector = ArmEndDetector()
        self.filled = True

    def process_pic(self, img):
        self.cup_detector.scan_pic(img)
        print(self.cup_detector.known_position)

