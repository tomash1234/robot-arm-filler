import numpy as np
import cv2
from abstract_detector import Detector
from utils import draw_circles


class ArmEndDetector(Detector):

    def __init__(self):
        super(ArmEndDetector, self).__init__(time_threshold_lost=2, time_threshold_tracked=2, distance_threshold=300)
        self.MAX_RADIUS = 30
        self.MIN_RADIUS = 8
        self.EDGE_THR = 200
        self.CIRCLE_THR = 40

    def process(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = cv2.medianBlur(gray, 5)
        marker = self.detect_marker(gray, img)
        self.compare_with_last_pos(marker)
        self.last_pos = marker

    def detect_marker(self, gray, img):
        rows = gray.shape[0]
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, rows / 8,
                                   param1=self.EDGE_THR, param2=self.CIRCLE_THR,
                                   minRadius=self.MIN_RADIUS, maxRadius=self.MAX_RADIUS)
        if circles is None:
            return None

        draw_circles(img, circles, (0, 255, 0))
        return circles[0][0]
