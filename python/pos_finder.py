"""Position finder module

This module implements basic algorithm to find relative location of cup suspect to roboarm.
Two basis vectors are estimated using moving arm in two directions
and calculating difference in image.

  https://github.com/tomash1234/robot-arm-filler
"""

import numpy as np
import time

"""States of estimating position"""
PREPARE = 0
DER_X = 1
DER_Y = 2

"""Delta in same units as arm dimensions (centimeters in my case)"""
DELTA = 2


def check_distance(marker, cup, thr):
    """Returns distance between cup and marker.
    If distance is less than threshold, it returns 0.

    Args:
        marker: (x, y, z) position of marker
        cup: (x, y, z) position of cup
        thr: float, threshold
    Returns:
        distance in same units as arm dimensions
    """
    marker = np.array([marker[:2]])
    cup = np.array([cup[:2]])
    dist = np.linalg.norm(marker - cup)
    if dist < thr:
        return 0

    return dist


class PosFinder:

    """Class to estimate relative position of cup suspect to robo arm poisiton"""

    def __init__(self, init_pose):
        """Inits Pos Finder class with initial position
        Args:
            init_pose   (x, y, z) init position
        """
        self.last_pos = init_pose
        self.change = np.array([0, 0])
        self.state = PREPARE
        self.x0 = init_pose
        self.dx = None
        self.dy = None
        self.cup0 = np.array([0, 0])
        self.marker0 = np.array([0, 0])
        self.last_time = time.time()
        self.last_reachable_pos = self.last_pos

    def iteration(self, cup, marker, valid):
        """Call thhis method every iteration
        Args:
            cup:    (x, y) position of cup in the image
            marker:    (x, y) position of marker in the image
            valid:  boolean if the previous position was valid
        """
        if valid:
            self.last_reachable_pos = self.last_pos

        if time.time() - self.last_time < 1:
            return None, False

        self.last_time = time.time()
        error = check_distance(marker, cup, cup[2] * 0.95)
        if error == 0:
            return self.last_pos, True

        return self.estimate_vectors(cup, marker), False

    def estimate_vectors(self, cup, marker):
        """Estimate basis vector by moving arm into two direction and calculating
         the difference between 2d position detected in the image .
        Args:
            cup:    (x, y) position of cup in the image
            marker:    (x, y) position of marker in the image
        Returns:
            Returns 3D position (x, y, z) where the arm should move next time
        """
        if self.state == PREPARE:
            self.cup0 = np.array([cup[0], cup[1]])
            self.marker0 = np.array([marker[0], marker[1]])
            self.x0 = self.last_pos
            self.state = DER_X
            return self.x0[0] + DELTA, self.x0[1], self.x0[2]
        if self.state == DER_X:
            m = np.array([marker[0], marker[1]])
            self.dx = m - self.marker0
            self.state = DER_Y
            return self.x0[0], self.x0[1], self.x0[2] + DELTA
        if self.state == DER_Y:
            m = np.array([marker[0], marker[1]])
            self.dy = m - self.marker0
            self.state = PREPARE

            return self.quick_pos(self.cup0, self.marker0)

    def quick_pos(self, cup, marker):
        """Calculate position after estimating basis vectors
        Args:
            cup:    (x, y) position of cup in the image
            marker:    (x, y) position of marker in the image
        Returns:
            Returns 3D position (x, y, z) where the arm should move
        """
        cup0 = np.array([cup[0], cup[1]])
        marker0 = np.array([marker[0], marker[1]])
        diff = cup0 - marker0
        a = np.array([self.dx, self.dy])
        res = np.linalg.solve(a, diff)
        new_x = np.array([self.x0[0], self.x0[2]]) + res * DELTA

        self.last_pos = (new_x[0], self.x0[1], new_x[1])
        return self.last_pos

    def reset(self, init_pose):
        """Reset state to the initial state
        Args:
            init_pose:  (x, y, z), initial position
        """
        self.last_pos = init_pose
        self.change = np.array([0, 0])
        self.state = PREPARE
        self.x0 = init_pose
        self.dx = None
        self.dy = None
        pass
