import numpy as np
import time

PREPARE = 0
DER_X = 1
DER_Y = 2
DELTA = 0.5


def check_distance(marker, cup, thr):
    marker = np.array([marker[:2]])
    cup = np.array([cup[:2]])
    dist = np.linalg.norm(marker - cup)
    if dist < thr:
        return 0

    return dist


class PosFinder:

    def __init__(self, init_pose):
        self.last_pos = init_pose
        self.change = np.array([0, 0])
        self.state = PREPARE
        self.x0 = init_pose
        self.dx = 0
        self.dy = 0
        self.er0 = 0
        self.last_time = time.time()
        self.last_reachable_pos = self.last_pos

    def iteration(self, cup, marker, valid):
        if valid:
            self.last_reachable_pos = self.last_pos
        else:
            self.last_pos = self.last_reachable_pos

        if time.time() - self.last_time < 1:
            return None

        self.last_time = time.time()
        thr = cup[2] * 0.75

        error = check_distance(marker, cup, thr)
        if error == 0:
            return self.last_pos

        if self.state == PREPARE:
            self.er0 = error
            self.x0 = self.last_pos
            self.state = DER_X
            return self.x0[0] + DELTA, self.x0[1], self.x0[2]
        if self.state == DER_X:
            self.dx = (error - self.er0) / DELTA * 2
            self.state = DER_Y
            return self.x0[0], self.x0[1], self.x0[2] + DELTA
        if self.state == DER_Y:
            self.dy = (error - self.er0) / DELTA * 2
            self.state = PREPARE
            self.last_pos = self.x0[0] - self.er0 / self.dx, self.x0[1], self.x0[2]-self.er0 / self.dy
            return self.last_pos
