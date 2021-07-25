import numpy as np

from arm_controller import ArmDriver
from arm_viewer import ArmViewer
from arm_dimensions import ArmDimensionsJson
from arm_controller import ArmCommunicator


if __name__ == '__main__':
    dim = ArmDimensionsJson('config.json')

    communicator = ArmCommunicator('192.168.137.156', 5101)
    communicator.set_pump_ip('192.168.137.47', 5101)
    communicator.send_pump(False)
    driver = ArmDriver(dim)
    viewer = ArmViewer(dim, driver, communicator)






