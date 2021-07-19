import numpy as np


def create_rot_x(theta):
    return np.matrix([[1, 0, 0, 0],
                      [0, np.cos(theta), -np.sin(theta), 0],
                      [0, np.sin(theta), np.cos(theta), 0],
                      [0, 0, 0, 1]])


def create_rot_y(theta):
    return np.matrix([[np.cos(theta), 0, np.sin(theta), 0],
                      [0, 1, 0, 0],
                      [-np.sin(theta), 0, np.cos(theta), 0],
                      [0, 0, 0, 1]])


def create_rot_z(theta):
    return np.matrix([[np.cos(theta), -np.sin(theta), 0, 0],
                      [np.sin(theta), np.cos(theta), 0, 0],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])


def create_tran(x, y, z):
    return np.matrix([[1, 0, 0, x],
                      [0, 1, 0, y],
                      [0, 0, 1, z],
                      [0, 0, 0, 1]])


def convert_point(point, which):
    if which == 'xy':
        return np.array([point[0], point[1]])
    if which == 'xz':
        return np.array([point[0], point[2]])
    if which == 'yz':
        return np.array([point[1], point[2]])
    return point


class Kinematics:

    def __init__(self, dimensions):
        self.base_off = np.identity(4)
        self.arm_off = np.identity(4)
        self.elbow_off = np.identity(4)
        self.forearm_off = np.identity(4)
        self.base_rot = np.identity(4)
        self.shoulder_rot = np.identity(4)
        self.elbow_rot = np.identity(4)

        self.setup_offset(dimensions)

    def setup_offset(self, dim):
        self.base_off = create_tran(dim.shoulder_o, dim.base_h, dim.shoulder_h_o)
        self.arm_off = create_tran(dim.arm_l, 0, 0)
        self.elbow_off = create_tran(0, dim.elbow_o, dim.elbow_h_o)
        self.forearm_off = create_tran(dim.elbow_l, 0, 0)

    def calculate(self, base, shoulder, elbow):
        p = np.array([0, 0, 0, 1])

        base_rot = create_rot_y(base)
        shoulder_rot = create_rot_z(shoulder)
        forearm_rot = create_rot_z(elbow)

        base_trans = base_rot @ self.base_off
        elbow_trans = base_trans + base_rot @ shoulder_rot @ self.arm_off @ self.elbow_off
        end_trans = elbow_trans + base_rot @ shoulder_rot @ forearm_rot @ self.forearm_off

        shoulder_pos = base_trans @ p
        arm_end_pos = (base_trans + base_rot @ shoulder_rot @ self.arm_off) @ p
        elbow_pos = elbow_trans @ p
        end_pos = end_trans @ p
        return [np.asarray(shoulder_pos[0, 0:3]).reshape(-1), \
               np.asarray(arm_end_pos[0, 0:3]).reshape(-1), \
               np.asarray(elbow_pos[0, 0:3]).reshape(-1), \
               np.asarray(end_pos[0, 0:3]).reshape(-1)]

    def calculate_shoulder_pos(self, base, which='all'):
        p = np.array([0, 0, 0, 1])
        base_rot = create_rot_y(base)
        base_trans = base_rot @ self.base_off
        shoulder_pos = base_trans @ p
        shoulder_pos = np.asarray(shoulder_pos[0, 0:3]).reshape(-1)
        return convert_point(shoulder_pos, type)
