import numpy as np
import time

PREPARE = 0
DER_X = 1
DER_Y = 2
DELTA = 2


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
        self.dx = None
        self.dy = None
        self.cup0 = np.array([0, 0])
        self.marker0 = np.array([0, 0])
        self.last_time = time.time()
        self.last_reachable_pos = self.last_pos

    def iteration(self, cup, marker, valid):
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
        cup0 = np.array([cup[0], cup[1]])
        marker0 = np.array([marker[0], marker[1]])
        diff = cup0 - marker0
        a = np.array([self.dx, self.dy])
        res = np.linalg.solve(a, diff)
        new_x = np.array([self.x0[0], self.x0[2]]) + res * DELTA

        self.last_pos = (new_x[0], self.x0[1], new_x[1])
        return self.last_pos

    def reset(self, init_pose):
        self.last_pos = init_pose
        self.change = np.array([0, 0])
        self.state = PREPARE
        self.x0 = init_pose
        self.dx = None
        self.dy = None
        pass
