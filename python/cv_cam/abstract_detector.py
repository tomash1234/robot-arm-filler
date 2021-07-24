import time
import numpy as np


class Detector:

    def __init__(self, time_threshold, distance_threshold):
        self.time_threshold = time_threshold
        self.distance_threshold = distance_threshold
        self.last_pos = (0, 0, 0)
        self.last_valid_pos = (0, 0, 0)
        self.last_time = time.time()
        self.last_detected = time.time()
        self.known_position = False

    def compare_with_last_pos(self, cup):
        if self.last_pos is None:
            return
        if cup is None:
            if time.time() - self.last_detected > self.time_threshold * 2:
                self.known_position = False
            return

        a = np.array(cup[:2])
        b = np.array(self.last_pos[:2])
        dist = np.linalg.norm(a - b)

        if dist > self.distance_threshold:
            self.last_time = time.time()
            self.known_position = False
        elif time.time() - self.last_time > self.time_threshold:
            self.known_position = True
        else:
            self.last_detected = time.time()

    def is_tracked(self):
        return self.known_position

    def get_pos(self):
        return self.last_valid_cup_pos