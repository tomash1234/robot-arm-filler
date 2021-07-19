import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

import matplotlib.patches as patches
from arm_controller import convert_point_2_local_slice, convert_local_slice_2_point


class ArmViewer:

    def __init__(self, dim, driver, communicator):
        self.dim = dim
        self.com = communicator
        self.driver = driver

        fig, (self.ax2, self.ax) = plt.subplots(1, 2)
        plt.subplots_adjust(bottom=0.25)

        self.ax.axis('equal')
        self.ax2.axis('equal')
        self.clear_side_view()
        self.clear_top_view()
        self.base_angle = None
        self.shoulder_angle = None
        self.elbow_angle = None
        self.click_x = 0
        self.click_y = 0
        self.click_z = 0
        self.base_angle_val = 0
        self.raw_angle = 0

        self.init_sliders(fig)

        def onclick(event):
            ix, iy = event.xdata, event.ydata
            if self.driver is not None and ix is not None:
                if event.inaxes == self.ax:
                    self.click_y = iy
                    point = convert_local_slice_2_point(self.raw_angle, ix, iy)
                elif event.inaxes == self.ax2:
                    self.click_x = ix
                    self.click_z = iy
                    point = np.array([self.click_x, self.click_y, self.click_z])
                else:
                    return

                self.target_new_point(point)

        fig.canvas.mpl_connect('button_press_event', onclick)
        plt.show()

    def target_new_point(self, point):
        ret = self.driver.find_angles(point)
        if ret is not None:
            angles = ret['angles']
            self.base_angle_val = angles[0]
            self.raw_angle = ret['raw_base']
            self.set_slider_values(angles)
            self.robot_arm_value_changed(angles)

    def set_slider_values(self, angles):
        self.slider_base.valinit = angles[0]
        self.slider_base.reset()
        self.slider_shoulder.valinit = angles[1]
        self.slider_shoulder.reset()
        self.slider_elbow.valinit = angles[2]
        self.slider_elbow.reset()

    def robot_arm_value_changed(self, angles):
        self.com.send_angles(self.driver.convert_angles(angles))
        points = self.driver.get_points(angles)

        side_points = self.driver.get_points([0, angles[1], angles[2]])
        side_points[0] = convert_point_2_local_slice(side_points[0], angles[0])
        self.set_disp_angles('{:0.1f}'.format(angles[0]), '{:0.1f}'.format(angles[1]),
                             '{:0.1f}'.format(angles[2]))
        self.draw_side_view(side_points[0], side_points[1], side_points[2], side_points[3])
        self.draw_top_view(points[0], points[1], points[2], points[3])

    def init_sliders(self, fig):
        base_joint = self.dim.get_joint(0)
        ax_slider_base = fig.add_axes([0.15, 0.16, 0.30, 0.025])
        self.slider_base = Slider(ax_slider_base, 'Base', base_joint.min, base_joint.max)

        shoulder_joint = self.dim.get_joint(1)
        ax_slider_shoulder = fig.add_axes([0.15, 0.125, 0.30, 0.025])
        self.slider_shoulder = Slider(ax_slider_shoulder, 'Shoulder', shoulder_joint.min,
                                      shoulder_joint.max, facecolor=(0.8, 0.2, 0.1))

        shoulder_joint = self.dim.get_joint(2)
        ax_slider_elbow = fig.add_axes([0.15, 0.09, 0.30, 0.025])
        self.slider_elbow = Slider(ax_slider_elbow, 'Elbow', shoulder_joint.min, shoulder_joint.max,
                                   facecolor=(0.2, 0.8, 0.3))

        def update(val):
            angles = [self.slider_base.val, self.slider_shoulder.val, self.slider_elbow.val]
            self.robot_arm_value_changed(angles)
        self.slider_base.on_changed(update)
        self.slider_shoulder.on_changed(update)
        self.slider_elbow.on_changed(update)

    def clear_side_view(self):
        self.ax.clear()
        self.set_limits_1()
        self.ax.set_title('Side View')

    def set_limits_1(self):
        max_x = self.dim.arm_l + self.dim.elbow_l
        self.ax.set_xlim(-self.dim.shoulder_o * 3 - max_x, max_x * 1.1)
        self.ax.set_ylim(-max_x, max_x * 2 + self.dim.base_h)

    def set_limits_2(self):
        max_x = self.dim.arm_l + self.dim.elbow_l
        self.ax2.set_xlim(-max_x*1.2, max_x * 1.2)
        self.ax2.set_ylim(-max_x*1.2, max_x*1.2)

    def clear_top_view(self):
        self.ax2.clear()
        self.set_limits_2()
        self.ax2.set_title('Top View')

    def draw_side_view(self, shoulder_pos, end_arm, elbow_pos, end_pos):
        shoulder_pos = shoulder_pos[0:2]
        elbow_pos = elbow_pos[0:2]
        end_pos = end_pos[0:2]
        self.clear_side_view()
        self.ax.add_patch(
            patches.Rectangle(
                (-2.5, 0),
                5, self.dim.base_h,
                fill=(0.1, 0.1, 0.1),
            ))

        self.ax.plot([shoulder_pos[0], end_arm[0]], [shoulder_pos[1], end_arm[1]], linewidth=5,
                     color=(0.8, 0.4, 0.4))
        self.ax.plot([end_pos[0], elbow_pos[0]], [end_pos[1], elbow_pos[1]], linewidth=5, color=(0.2, 0.8, 0.4))
        self.ax.plot(end_pos[0], end_pos[1], 'x', markersize=10, color='r')

        if self.base_angle is not None:
            self.ax.text(shoulder_pos[0], 2, self.base_angle, fontsize=8, backgroundcolor='blue',
                         color='white')
            self.ax.text(shoulder_pos[0], shoulder_pos[1]+3, self.shoulder_angle, fontsize=8,
                         backgroundcolor=(1, 0, 0.0, 0.4),
                         color='black')
            self.ax.text(elbow_pos[0], elbow_pos[1], self.elbow_angle, fontsize=8,
                         backgroundcolor=(0.0, 1, 0.0, 0.4),
                         color='black')
        self.set_limits_1()

    def draw_top_view(self, shoulder_pos, end_arm, elbow_pos, end_pos):
        shoulder_pos = [shoulder_pos[0], shoulder_pos[2]]
        elbow_pos = [elbow_pos[0], elbow_pos[2]]
        end_pos = [end_pos[0], end_pos[2]]
        self.clear_top_view()

        self.ax2.add_patch(
            patches.Rectangle(
                (-2.5, -1.5),
                5, 3,
                fill=(0.1, 0.1, 0.1),
            ))

        self.ax2.plot([shoulder_pos[0], end_arm[0]], [shoulder_pos[1], end_arm[2]], linewidth=5,
                     color=(0.8, 0.4, 0.4))
        self.ax2.plot([end_pos[0], elbow_pos[0]], [end_pos[1], elbow_pos[1]], linewidth=5, color=(0.2, 0.8, 0.4))
        self.ax2.plot(end_pos[0], end_pos[1], 'x', markersize=10, color='r')
        self.set_limits_2()
        plt.show()

    def set_disp_angles(self, base, shoulder, elbow):
        self.base_angle = base
        self.shoulder_angle = shoulder
        self.elbow_angle = elbow
