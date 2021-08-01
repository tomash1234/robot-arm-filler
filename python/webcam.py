"""Webvam module

  Run this module if you want to start auto-filler with a webcam

  https://github.com/tomash1234/robot-arm-filler
"""

import cv2
from matplotlib import pyplot as plt
from filling_manager import FillingManager


def show_webcam(mirror=False, device=0):
    """Start a webcam
    Args:
        mirror: boolean, should be image inverted in x axis
        device: webcam device id
    """
    plt.subplot(1, 2, 1)
    cam = cv2.VideoCapture(device)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    filling_manger = FillingManager('192.168.137.75', 5101)
    filling_manger.set_pump_address('192.168.137.139', 5101)

    while True:
        ret_val, img = cam.read()
        if mirror:
            img = cv2.flip(img, 1)

        img = cv2.resize(img, (640, 360))
        filling_manger.process_pic(img)
        cv2.imshow('Robot Auto Filler', img)
        if cv2.waitKey(1) == 27:
            break  # esc to quit
    cv2.destroyAllWindows()


show_webcam(mirror=True)