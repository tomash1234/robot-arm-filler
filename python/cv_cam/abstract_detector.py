import time
import numpy as np


class Detector:

    def __init__(self, time_threshold_tracked, time_threshold_lost, distance_threshold):
        self.time_threshold_tracked = time_threshold_tracked
        self.time_threshold_lost = time_threshold_lost
        self.distance_threshold = distance_threshold
        self.last_pos = (0, 0, 0)
        self.last_valid_pos = (0, 0, 0)
        self.last_time = time.time()
        self.last_detected = time.time()
        self.known_position = False

    def compare_with_last_pos(self, pos):
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
        return self.known_position

    def get_pos(self):
        return self.last_valid_pos