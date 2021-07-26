"""
RoboArm Dimension Module

 This module contains classes representing physical dimensions of a robotic arm.

 https://github.com/tomash1234/robot-arm-filler
"""

import numpy as np
import json


class ArmJoint:
    """"This class represents one servo motor

    Attributes:
        min:    an integer minimal angle which the servo can move to
        max:    an integer maximal angle which the servo can move to
        inv:    a boolean True when the angles should be inverted
        cor:    an integer correction angle which is added to virtual angles

    """

    def __init__(self, data):
        """Inits Joint Class with data from json file

        Args:
            data:    a dictionary containing following parameters:
                    `min`, `max`, `inverse` and `cor`
        """
        self.min = data['min']
        self.max = data['max']
        self.inv = True if data['inverse'] == 1 else False
        self.cor = data['cor']

    def convert_angle(self, angle):
        """Converts virtual angle to the angle for servo

        Virtual angle is the angle used in kinematics function, which is from -90 to 90 degrees.
        However, the servo motors operate from [min] to [max] degrees. Robotics arm servo doesnt have to
        be perfectly align, so there is a correction parameter which can compensate this.
        Sometimes the angle needs to be inverted.

        Args:
             angle: float, virtual angle in degrees

        Returns:
            Returns integer, angle converted for specific physical servo

        """
        if self.inv:
            angle *= -1
        angle = int(angle + self.cor)
        angle = min(180, max(angle, 0))
        return angle

    def is_valid_angle(self, angle):
        """Check if the angle is valid

        Args:
            angle:  float, angle to check

        Returns:
            boolean. True if the angle is in the specific range

        """
        return self.min <= angle <= self.max


class ArmDimensions:
    """Class storing dimensions of robot arm parts

    For further parts description see readme and picture in doc folder.

    Attributes:
        base_b:     a float representing the base
        shoulder_o:
        arm_l:
        shoulder offset:
        shoulder_h_o:
        elbow_h_o:
        joints:
    """
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
        """Returns Arm Joint instance

        Args:
            index:  index of the joint

        Returns:
            An instance of ArmJoint describing the specific joint
        """
        return self.joints[index]


class ArmDimensionsJson(ArmDimensions):
    """ Class to load the arm dimensions from config JSON file

    Arm dimensions can be loaded from config file in JSON format.
    """

    def __init__(self, config_file):
        """Inits the instance with data from a config file

        Args:
            config_file:    path to config JSON file

        """
        file = open(config_file)
        data = json.load(file)
        file.close()

        joints = []
        for data_joint in data['joints']:
            joints.append(ArmJoint(data_joint))

        super(ArmDimensionsJson, self).__init__(data['baseHeight'], data['shoulderOffsetX'], data['shoulderOffsetZ'],
                                                data['armLength'], data['elbowOffsetX'], data['elbowOffsetZ'],
                                                data['elbowLength'], joints)
