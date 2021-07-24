import numpy as np
import cv2
from abstract_detector import Detector


class ArmEndDetector(Detector):

    def __init__(self):
        super(ArmEndDetector, self).__init__(time_threshold_lost=2, time_threshold_tracked=2, distance_threshold=300)
        pass

    def process(self, img):
        pass
