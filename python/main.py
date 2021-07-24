import numpy as np

from arm_controller import ArmDriver
from arm_viewer import ArmViewer
from arm_dimensions import ArmDimensionsJson
from arm_controller import ArmCommunicator


if __name__ == '__main__':
    dim = ArmDimensionsJson('config.json')

    communicator = ArmCommunicator('192.168.137.5', 5101)
    driver = ArmDriver(dim)
    viewer = ArmViewer(dim, driver, communicator)






