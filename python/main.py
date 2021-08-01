"""Main module

  Run this module if you want to start GUI Arm Viewer

  https://github.com/tomash1234/robot-arm-filler
"""

from matplotlib.widgets import Button

from arm_controller import ArmDriver
from arm_viewer import ArmViewer
from arm_dimensions import ArmDimensionsJson
from arm_controller import ArmCommunicator


def add_pump_button(view, com):
    """Add a pump button to the viewer"""
    ax_button = view.fig.add_axes([0.60, 0.10, 0.30, 0.06])
    button_pump = Button(ax_button, 'START PUMP')

    def on_pump_button_click(event):
        print('click')
        view.pump_running = not view.pump_running
        if view.pump_running:
            button_pump.label.set_text('STOP PUMP')
        else:
            button_pump.label.set_text('START PUMP')

        com.send_pump(view.pump_running)

    button_pump.on_clicked(on_pump_button_click)

    return button_pump


if __name__ == '__main__':
    dim = ArmDimensionsJson('config.json')

    communicator = ArmCommunicator('192.168.137.75', 5101)

    communicator.set_pump_ip('192.168.137.139', 5101)
    communicator.send_pump(False)

    driver = ArmDriver(dim)
    viewer = ArmViewer(dim, driver, communicator)

    but_pump = add_pump_button(viewer, communicator)

    viewer.show()
