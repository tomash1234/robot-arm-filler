import cv2
from matplotlib import pyplot as plt
from filling_manager import FillingManager


def show_webcam(mirror=False, device=0):
    plt.subplot(1, 2, 1)
    cam = cv2.VideoCapture(device)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    filling_manger = FillingManager()
    while True:
        ret_val, img = cam.read()
        if mirror:
            img = cv2.flip(img, 1)

        img = cv2.resize(img, (640, 360))
        filling_manger.process_pic(img)
        cv2.imshow('my webcam', img)
        if cv2.waitKey(1) == 27:
            break  # esc to quit
    cv2.destroyAllWindows()


show_webcam(mirror=True)