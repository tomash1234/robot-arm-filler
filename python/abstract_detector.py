# -*- coding: utf-8 -*-
"""
Abstract Detector module

 This module contains Abstract Detector class which is used by CupDetector
 and ArmEndDetector. It is implemented basic logic which used time thresholds
 to determined whether the object is detected or not.


 https://github.com/tomash1234/robot-arm-fillter
"""

import time
import numpy as np


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
