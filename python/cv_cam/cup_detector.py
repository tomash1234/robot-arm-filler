import numpy as np
import cv2
from abstract_detector import Detector


def draw_circles(image, circles):
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            center = (i[0], i[1])
            cv2.circle(image, center, 1, (255, 0, 0), 3)
            radius = i[2]
            cv2.circle(image, center, radius, (255, 255, 0), 3)


class CupDetector(Detector):

    def __init__(self):
        super(CupDetector, self).__init__(time_threshold_lost=2, time_threshold_tracked=2, distance_threshold=300)
        self.MAX_RADIUS = 250
        self.MIN_RADIUS = 50
        self.EDGE_THR = 300
        self.CIRCLE_THR = 50

    def scan_pic(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = cv2.medianBlur(gray, 5)

        cup = self.detect_cup(gray)
        self.compare_with_last_pos(cup)
        self.last_pos = cup
        if cup is None:
            return

        self.estimate_fullness(img, cup)
        draw_circles(img, [[cup]])
        return cup

    def detect_cup(self, gray):
        rows = gray.shape[0]
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, rows / 8,
                                   param1=self.EDGE_THR, param2=self.CIRCLE_THR,
                                   minRadius=self.MIN_RADIUS, maxRadius=self.MAX_RADIUS)
        if circles is None:
            return None
        return circles[0][0]

    def estimate_fullness(self, img, cup):
        x, y, r = cup
        x = int(x)
        y = int(y)
        r = int(r)
        if r < 1 or x - r < 1 or x + r > len(img[0]) or\
                y - r < 1 or y + r > len(img[0]):
            return None
        crop = img[y - r: y + r, x - r: x + r]

        cv2.imshow('Crop', crop)
