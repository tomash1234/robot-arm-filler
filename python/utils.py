import numpy as np
import cv2


def draw_circles(image, circles, color=(255, 255, 0)):
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            center = (i[0], i[1])
            cv2.circle(image, center, 1, (255, 0, 0), 3)
            radius = i[2]
            cv2.circle(image, center, radius, color, 3)
