import numpy as np
import json


class ArmJoint:

    def __init__(self, data):
        self.min = data['min']
        self.max = data['max']
        self.inv = True if data['inverse'] == 1 else False
        self.cor = data['cor']

    def convert_angle(self, angle):
        if self.inv:
            angle *= -1
        angle = int(angle + self.cor)
        angle = min(180, max(angle, 0))
        return angle

    def is_valid_angle(self, angle):
        return self.min <= angle <= self.max


class ArmDimensions:
    def __init__(self, base_height, shoulder_offset, shoulder_h_o, arm_len, elbow_offset, elbow_h_o, elbow_len, joints):
        self.base_h = base_height
        self.shoulder_o = shoulder_offset
        self.arm_l = arm_len
        self.elbow_o = elbow_offset
        self.elbow_l = elbow_len
        self.arm_radius = np.sqrt(arm_len ** 2 + elbow_offset ** 2)
        self.shoulder_corr_angle = np.arctan(self.elbow_o / arm_len)
        self.shoulder_h_o = shoulder_h_o
        self.elbow_h_o = elbow_h_o
        self.joints = joints

    def get_joint(self, index):
        return self.joints[index]


class ArmDimensionsJson(ArmDimensions):

    def __init__(self, config_file):
        file = open(config_file)
        data = json.load(file)
        file.close()

        joints = []
        for data_joint in data['joints']:
            joints.append(ArmJoint(data_joint))

        super(ArmDimensionsJson, self).__init__(data['baseHeight'], data['shoulderOffsetX'], data['shoulderOffsetZ'],
                                                data['armLength'], data['elbowOffsetX'], data['elbowOffsetZ'],
                                                data['elbowLength'], joints)
