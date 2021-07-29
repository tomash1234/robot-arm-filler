"""
Arm Controller Module

    This module contains classes to communication with the board and basic inverse kinematics.

    https://github.com/tomash1234/robot-arm-filler
"""

import socket
import numpy as np

from kinematics import Kinematics

""" Servo motors indexes """
SERVO_BASE = 0
SERVO_SHOULDER = 1
SERVO_ELBOW = 2


class ArmCommunicator:
    """ Arm Communicator allows to send UDP packets to the board.

    Attributes:
        address:    string, IP address of the board
        port:       integer, board port for communication
        address2:   string, IP address of the second board (pump controller)
        port2:      integer, board port for communication with the second board (pump controller)

        socket:     socket for UPD communication
    """

    def __init__(self, ip_address, port=5051):
        """ Inits the instance with ip address of board and its port"""
        self.address = ip_address
        self.port = port
        self.address2 = None
        self.port2 = None

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def set_pump_ip(self, ip_address, port=5051):
        """Sets the second board (pump controller) address and its port"""
        self.address2 = ip_address
        self.port2 = port

    def send_pump(self, on):
        """Starts / Stops pump. Sends UDP packet to the second board
        Args:
            on:     boolean, True turns the pump on and False turns it off
        """
        if self.address2 is None:
            return
        mes = [255 if on else 0]
        print(mes, self.address2, self.port2)
        self.socket.sendto(bytes(mes), (self.address2, self.port2))

    def send_angles(self, angles):
        """Sends all angles to the board which should set the servos
         to the specific angles. Angles are in degrees in the range [0, 255]

         Args:
             angles:    list of 3 integers, [base angle, shoulder angle, elbow angle] in degrees
         """
        self.send_packet(angles)

    def send_packet(self, message):
        """Sends message to the first board (address and port from contractor)
        Args:
            message:    list of byte values
        """
        self.socket.sendto(bytes(message), (self.address, self.port))


def find_circles_interceptions(c1, r1, c2, r2):
    """Finds interceptions of two circles.

    Args:
        c1: center of the first circle (x, y)
        r1: float, radius of the first circle
        c2: center of the second circle (x, y)
        r2: float, radius of the second circle

    Returns:
        list of interceptions or None if there is no interceptions

    """
    R = np.linalg.norm(c2 - c1)
    if R > r1 + r2:
        return None
    f = (c1 + c2) / 2
    diff = c1 - c2

    res = f + (c2 - c1) * (r1 ** 2 - r2 ** 2) / (2 * R ** 2)
    g = (R + r1 + r2) * (R + r1 - r2) * (R - r1 + r2) * (- R + r1 + r2)
    if g < 0:
        return None
    e = 0.25 * np.sqrt(g)
    t = 2 * e * (np.array([diff[1], diff[0]])) / R ** 2

    inters = [[res[0] - t[0], res[1] + t[1]], [res[0] + t[0], res[1] - t[1]]]

    return inters


def angles_bet_vec(u, v):
    """Find signed angle between two vectors
    Args:
        u:  vector 1
        v:  vector 2

    Returns:
        float, signed angle in radians

    """
    if u[1] - v[1] == 0:
        sign = 1
    else:
        sign = (v[1] - u[1]) / np.abs(v[1] - u[1])
    c = np.dot(u, v) / (np.linalg.norm(u) * np.linalg.norm(v))
    return sign * np.arccos(c)


def convert_point_2_local_slice(point, cor_angle):
    r = 1 * (point[0] ** 2 + point[2] ** 2) ** 0.5
    return np.array([r - np.cos(np.deg2rad(cor_angle)), point[1]])


def convert_local_slice_2_point(raw_angle, x, y):
    new_point = np.array([x * np.cos(np.deg2rad(raw_angle)), y,
                          -x * np.sin(np.deg2rad(raw_angle))])
    return new_point


class ArmDriver:

    """ Arm Driver class implements basic inverse kinematics function
        and also calculates position of arm end based on given angles.

    Attributes:
        dim:    ArmDimensions, dimensions of arm
        kim:    Kinematics, instance of kinematics class
    """

    def __init__(self, dimensions):
        """Inits Arm Driver instance from given dimensions"""
        self.dim = dimensions
        self.kim = Kinematics(dimensions)

    def find_valid_angle(self, interceptions, point, shoulder, base_angle):
        if not self.dim.get_joint(0).is_valid_angle(base_angle):
            print('No valid angles b')
            return None

        arm = [interceptions[0] - shoulder[0:2], interceptions[1] - shoulder[0:2]]
        shoulder_angles = [angles_bet_vec(np.array([1, 0]), arm[0]), angles_bet_vec(np.array([1, 0]), arm[1])]
        shoulder_angles_deg = np.rad2deg(shoulder_angles - self.dim.shoulder_corr_angle)

        valid_indexes = [1, 0]
        # Check if angle is invalid
        shoulder_valid = []
        for i in valid_indexes:
            if self.dim.get_joint(1).is_valid_angle(shoulder_angles_deg[i]):
                shoulder_valid.append(i)

        if len(valid_indexes) == 0:
            print('No valid angles s')
            return None

        forearm = [point[:2] - interceptions[0], point[:2] - interceptions[1]]
        elbow_angles = [angles_bet_vec(arm[0], forearm[0]), angles_bet_vec(arm[1], forearm[1])]
        elbow_angles_deg = np.rad2deg(elbow_angles)

        all_valid = []
        for i in shoulder_valid:
            if self.dim.get_joint(2).is_valid_angle(elbow_angles_deg[i]):
                all_valid.append(i)

        if len(all_valid) == 0:
            print('No valid angles e')
            return None

        chosen_index = all_valid[0]
        return base_angle, shoulder_angles_deg[chosen_index], elbow_angles_deg[chosen_index], interceptions[
            chosen_index]

    def get_points(self, angles):
        """Returns the list of 3D points corresponding with
            shoulder position, end of arm, elbow position and tip of roboarm
        Args:
              angles:   list of 3 angles in degress [base_angle, shoulder angle, elbow angle]
        Returns:
              list of 3D position of shoulder pos, arm end, elbow and end of roboarm
        """
        base = np.deg2rad(angles[0])
        shoulder = np.deg2rad(angles[1])
        elbow = np.deg2rad(angles[2])
        return self.kim.calculate(base, shoulder, elbow)

    def find_base_rotation(self, point):
        """Find angle of base servo to reach point
        Args:
            point:  destination point
        Returns:
            angle in degrees with and without correction
            (angle + correction, angle)
        """
        point2d = np.array([point[0], point[2]])
        dis_x = self.dim.shoulder_h_o + self.dim.elbow_h_o
        correction_angle = np.arctan(dis_x / np.linalg.norm(point2d))
        angle = np.arctan2(-point[2], point[0])
        deg = np.rad2deg(angle + correction_angle)
        return deg, np.rad2deg(angle)

    def find_angles_with_threshold(self, point, threshold_y):
        """Find all 3 angles to reach destination point but allows threshold in y axis
        Args:
            point:  (x, y, z) destination point
            threshold_y:    float, threshold in y axis
        Returns:
            Dictionary where the keys are angles, shoulder, raw_angle and values are the angles,
            shoulder position and raw angle withou correction
        """
        ret = self.find_angles(point)
        if ret is None:
            p = (point[0], point[1] - threshold_y, point[2])
            ret = self.find_angles(p)

        if ret is None:
            p = (point[0], point[1] + threshold_y, point[2])
            ret = self.find_angles(p)

        return ret

    def find_angles(self, point):
        """Finds angles for all 3 servos to reach the destination point
        Args:
            point:  (x, y, z) destination point
        Returns:
            Dictionary where keys are angles, shoulder, raw_angle and values
            are a list of angles in degrees, shoulder position (x, y, z) and angle without correction

        """
        base_angle, raw_angle = self.find_base_rotation(point)
        shoulder_pos = self.kim.calculate_shoulder_pos(base_angle, 'xy')

        projected_start = np.array([self.dim.shoulder_o, self.dim.base_h])
        projected_point = convert_point_2_local_slice(point, base_angle - raw_angle)
        interceptions = find_circles_interceptions(projected_start,
                                                   self.dim.arm_radius,
                                                   projected_point, self.dim.elbow_l)

        if interceptions is None:
            print('Un reachable point |')
            return None

        ret = self.find_valid_angle(interceptions, projected_point, projected_start, base_angle)
        if ret is None:
            print('Un reachable point')
            return None
        angles = ret[:3]

        print('Found', angles)
        ret = {'angles': angles, 'shoulder': shoulder_pos, 'raw_base': raw_angle}
        return ret

    def convert_angles(self, angles):
        """Converts angles to angles ready to send into the board
        Args:
            angles: list of 3 float angles in degrees
        Returns:
             list of integer angles ready for the board
        """
        base = self.dim.get_joint(0).convert_angle(angles[0])
        shoulder = self.dim.get_joint(1).convert_angle(angles[1])
        elbow = self.dim.get_joint(2).convert_angle(angles[2])
        return [int(base), int(shoulder), int(elbow)]
