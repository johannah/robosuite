"""
Driver class for Keyboard controller.
"""

import glfw
import numpy as np
from robosuite.devices import Device
from robosuite.utils.transform_utils import rotation_matrix
from glob import glob
import pickle
from IPython import embed

class Mimic(Device):
    """
    A minimalistic driver class for a Mimic 

    Args:
        pos_sensitivity (float): Magnitude of input position command scaling
        rot_sensitivity (float): Magnitude of scale input rotation commands scaling
    """

    def __init__(self,
                 pos_sensitivity=1.0,
                 rot_sensitivity=1.0
                 ):

        #self._display_controls()
        self.mimic_from = '../../../frankmocap/output/jo_box3/mocap/'
        self.mimic_states = sorted(glob(self.mimic_from+'*.pkl'))
        self._reset_internal_state()

        self._reset_state = 0
        self._enabled = False
        self._pos_step = 0.05

        self.pos_sensitivity = pos_sensitivity
        self.rot_sensitivity = rot_sensitivity
        positions = []
        for i in range(len(self.mimic_states)):
            pkl = pickle.load(open(self.mimic_states[i], 'rb'))
            body_joints = pkl['pred_output_list'][0]['pred_joints_vis']
            right_shoulder = body_joints[33]
            right_wrist = body_joints[31]
            dpos = right_wrist - right_shoulder
            positions.append(dpos)
        self.positions = np.array(positions) 
        # axis x in human is left- to right+
        # axis x in robot is forward, back
        # axis y in human is down-, up+
        # axis y in robot is right-, left+
        # axis z in human is forward, away
        # axis z in robot is up+, down-
        self.positions = self.positions[:,[1,2,0]]
    @staticmethod
    def _display_controls():
        """
        Method to pretty print controls.
        """

    def _reset_internal_state(self):
        """
        Resets internal state of controller, except for the reset signal.
        """
        self.rotation = np.array([[-1., 0., 0.], [0., 1., 0.], [0., 0., -1.]])
        self.raw_drotation = np.zeros(3) # immediate roll, pitch, yaw delta values from keyboard hits
        self.last_drotation = np.zeros(3)
        self.pos = np.zeros(3)  # (x, y, z)
        self.last_pos = np.zeros(3)
        self.grasp = False
        self.cnt =  0

    def start_control(self):
        """
        Method that should be called externally before controller can
        start receiving commands.
        """
        self._reset_internal_state()
        self._reset_state = 0
        self._enabled = True

    def get_controller_state(self):
        """
        Grabs the current state of the keyboard.

        Returns:
            dict: A dictionary containing dpos, orn, unmodified orn, grasp, and reset
        """

        dpos = self.positions[self.cnt]
        self.cnt += 1
        return dict(
            dpos=dpos,
            rotation=self.rotation,
            raw_drotation=self.raw_drotation,
            grasp=int(self.grasp),
            reset=self._reset_state,
        )

#    def on_press(self, window, key, scancode, action, mods):
#        """
#        Key handler for key presses.
#
#        Args:
#            window: [NOT USED]
#            key (int): keycode corresponding to the key that was pressed
#            scancode: [NOT USED]
#            action: [NOT USED]
#            mods: [NOT USED]
#        """
#
#        # controls for moving position
#        if key == glfw.KEY_W:
#            self.pos[0] -= self._pos_step * self.pos_sensitivity  # dec x
#        elif key == glfw.KEY_S:
#            self.pos[0] += self._pos_step * self.pos_sensitivity   # inc x
#        elif key == glfw.KEY_A:
#            self.pos[1] -= self._pos_step * self.pos_sensitivity   # dec y
#        elif key == glfw.KEY_D:
#            self.pos[1] += self._pos_step * self.pos_sensitivity   # inc y
#        elif key == glfw.KEY_F:
#            self.pos[2] -= self._pos_step * self.pos_sensitivity   # dec z
#        elif key == glfw.KEY_R:
#            self.pos[2] += self._pos_step * self.pos_sensitivity   # inc z
#
#        # controls for moving orientation
#        elif key == glfw.KEY_Z:
#            drot = rotation_matrix(angle=0.1 * self.rot_sensitivity, direction=[1., 0., 0.])[:3, :3]
#            self.rotation = self.rotation.dot(drot)  # rotates x
#            self.raw_drotation[1] -= 0.1 * self.rot_sensitivity
#        elif key == glfw.KEY_X:
#            drot = rotation_matrix(angle=-0.1 * self.rot_sensitivity, direction=[1., 0., 0.])[:3, :3]
#            self.rotation = self.rotation.dot(drot)  # rotates x
#            self.raw_drotation[1] += 0.1 * self.rot_sensitivity
#        elif key == glfw.KEY_T:
#            drot = rotation_matrix(angle=0.1 * self.rot_sensitivity, direction=[0., 1., 0.])[:3, :3]
#            self.rotation = self.rotation.dot(drot)  # rotates y
#            self.raw_drotation[0] += 0.1 * self.rot_sensitivity
#        elif key == glfw.KEY_G:
#            drot = rotation_matrix(angle=-0.1 * self.rot_sensitivity, direction=[0., 1., 0.])[:3, :3]
#            self.rotation = self.rotation.dot(drot)  # rotates y
#            self.raw_drotation[0] -= 0.1 * self.rot_sensitivity
#        elif key == glfw.KEY_C:
#            drot = rotation_matrix(angle=0.1 * self.rot_sensitivity, direction=[0., 0., 1.])[:3, :3]
#            self.rotation = self.rotation.dot(drot)  # rotates z
#            self.raw_drotation[2] += 0.1 * self.rot_sensitivity
#        elif key == glfw.KEY_V:
#            drot = rotation_matrix(angle=-0.1 * self.rot_sensitivity, direction=[0., 0., 1.])[:3, :3]
#            self.rotation = self.rotation.dot(drot)  # rotates z
#            self.raw_drotation[2] -= 0.1 * self.rot_sensitivity
#
#    def on_release(self, window, key, scancode, action, mods):
#        """
#        Key handler for key releases.
#
#        Args:
#            window: [NOT USED]
#            key (int): keycode corresponding to the key that was pressed
#            scancode: [NOT USED]
#            action: [NOT USED]
#            mods: [NOT USED]
#        """
#
#        # controls for grasping
#        if key == glfw.KEY_SPACE:
#            self.grasp = not self.grasp  # toggle gripper
#
#        # user-commanded reset
#        elif key == glfw.KEY_Q:
#            self._reset_state = 1
#            self._enabled = False
#            self._reset_internal_state()
