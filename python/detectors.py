# -*- coding: utf-8 -*-
"""
 Detector module

 This module contains Abstract Detector class and following inherit detectors CupDetector
 and ArmEndDetector. It is implemented basic logic which used time thresholds
 to determined whether the object is detected or not.


 https://github.com/tomash1234/robot-arm-filler
"""

import time
import numpy as np
import cv2

from python.utils import draw_circles


class Detector:
    """ Base class for detector

        Attributes:
            time_threshold_tracked: A float time to consider the object detected, in seconds
            time_threshold_lost: A float time to consider the detected object is lost, in seconds
            distance_threshold: A float length threshold in pixels
        """

    def __init__(self, time_threshold_tracked, time_threshold_lost, distance_threshold):
        """ Inits basic detector with specific threshold values"""
        self.time_threshold_tracked = time_threshold_tracked
        self.time_threshold_lost = time_threshold_lost
        self.distance_threshold = distance_threshold
        self.last_pos = (0, 0, 0)
        self.last_valid_pos = (0, 0, 0)
        self.last_time = time.time()
        self.last_detected = time.time()
        self.known_position = False

    def compare_with_last_pos(self, pos):
        """Compare current position with previous.

        Comparing the position with the last one and based on the result
        updating current tracking state

        Args:
            pos: (x, y) position of the detected object

        """
        if pos is None:
            if time.time() - self.last_detected > self.time_threshold_lost:
                self.known_position = False
            return

        if self.last_pos is None:
            return

        a = np.array(pos[:2])
        b = np.array(self.last_pos[:2])
        dist = np.linalg.norm(a - b)

        if dist > self.distance_threshold:
            self.last_time = time.time()
            self.known_position = False
        elif time.time() - self.last_time > self.time_threshold_tracked:
            self.known_position = True
            self.last_detected = time.time()
            self.last_valid_pos = pos
        else:
            self.last_detected = time.time()
            self.last_valid_pos = pos

    def is_tracked(self):
        """ Returns true if the object is detected

        Returns
            bool - current state
        """
        return self.known_position

    def get_pos(self):
        """ Returns last valid position of the detected object

        Returns
            Last valid position (x, y)
        """
        return self.last_valid_pos


class ArmEndDetector(Detector):

    """Detector for finding end of the arm, where the small marker is located.

    This detector detects a simple black and white circular marker.
    In folder doc, there is a picture of the marker.

    A simple Hough circles detector is used

    Attributes:
          MAX_RADIUS: float, maximal radius of the marker in pixels
          MIN_RADIUS: float, minimal radius of the marker in pixels
          EDGE_THR: float,a canny edge threshold
          CIRCLE_THR: float, a circle threshold

    """

    def __init__(self):
        """Inits marker detector"""
        super(ArmEndDetector, self).__init__(time_threshold_lost=2, time_threshold_tracked=2, distance_threshold=300)
        self.MAX_RADIUS = 30
        self.MIN_RADIUS = 8
        self.EDGE_THR = 200
        self.CIRCLE_THR = 40

    def process(self, img):
        """This method is called every frame abd detect the marker

        Args:
            img:    cv2 image, image from webcam
        """
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = cv2.medianBlur(gray, 5)
        marker = self.detect_marker(gray, img)
        self.compare_with_last_pos(marker)
        self.last_pos = marker

    def detect_marker(self, gray, img):
        """Detects marker in gray picture and draw it in images
        SEE: https://docs.opencv.org/4.5.2/da/d53/tutorial_py_houghcircles.html

        Args:
            gray;   cv2 image, grayscale webcam image
            img:    cv2 image, color webcam image to draw detected circles

        Returns:
            Returns detected circle in following format (cx, cy, radius)
            if a circle is detected otherwise returns None
        """
        rows = gray.shape[0]
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, rows / 8,
                                   param1=self.EDGE_THR, param2=self.CIRCLE_THR,
                                   minRadius=self.MIN_RADIUS, maxRadius=self.MAX_RADIUS)
        if circles is None:
            return None

        draw_circles(img, circles, (0, 255, 0))
        return circles[0][0]


class CupDetector(Detector):
    """A detector for detecting the cup.

    Detector is implemented using a simple circle detector.
    A camera must view the cup from the top view.
    The cup must have a circular shape in the horizontal slice.

    Attributes:
          MAX_RADIUS: float, maximal radius of the marker in pixels
          MIN_RADIUS: float, minimal radius of the marker in pixels
          EDGE_THR: float,a canny edge threshold
          CIRCLE_THR: float, a circle threshold

    """

    def __init__(self):
        """Inits Cup detector"""
        super(CupDetector, self).__init__(time_threshold_lost=2, time_threshold_tracked=2, distance_threshold=300)
        self.MAX_RADIUS = 250
        self.MIN_RADIUS = 50
        self.EDGE_THR = 300
        self.CIRCLE_THR = 50

    def scan_pic(self, img):
        """Scan picture to find the cup in it and draw detected cup into the image

        Args:
            img:    cv2 image, image from the webcamera

        Returns:
            Detected cup circle in following format (cx, cy, radius)
             or None if there is no detected cup

        """
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
        """Detects marker in gray picture
        SEE: https://docs.opencv.org/4.5.2/da/d53/tutorial_py_houghcircles.html

        Args:
            gray;   cv2 image, grayscale webcam image

        Returns:
            Returns detected circle in following format (cx, cy, radius)
            if a circle is detected otherwise returns None
        """
        rows = gray.shape[0]
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, rows / 8,
                                   param1=self.EDGE_THR, param2=self.CIRCLE_THR,
                                   minRadius=self.MIN_RADIUS, maxRadius=self.MAX_RADIUS)
        if circles is None:
            return None
        return circles[0][0]

    def estimate_fullness(self, img, cup):
        """ TODO: Method should returns estimate fullness of the cup,
        however the methods is not implemented yet

        Args:
            img:    cv2 image, image from webcam
            cup:    (cx, cy, radius), detected cup

        Returns:
            Should return 0-1 float value representing fullness of the cup
            [or True and False Full / Empty]
        """
        x, y, r = cup
        x = int(x)
        y = int(y)
        r = int(r)
        if r < 1 or x - r < 1 or x + r > len(img[0]) or \
                y - r < 1 or y + r > len(img[0]):
            return None
        crop = img[y - r: y + r, x - r: x + r]

        cv2.imshow('Crop', crop)
