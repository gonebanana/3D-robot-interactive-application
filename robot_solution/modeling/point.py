import numpy as np
from PyQt5 import QtWidgets

from robot_solution.modeling.transform import myquat2rotm
from robot_solution.modeling.solution import solve_straight as solve_straight_imported, solve_forward


class Point:
    """ Class that describes point, given by user."""

    def __init__(self):
        # init point value corresponds to the init position of the robot with zero joint angles.
        self._pose = None  # x, y, z, quat1, quat2, quat3, quat4
        # look angles.setter
        self.angles = np.array([0] * 6)  # joint angles 1..6
        self.solved = False

    @property
    def pose(self):
        # x, y, z, quat1, quat2, quat3, quat4
        return self._pose

    @pose.setter
    def pose(self, value):
        self.try_solve_forward(value)

    @property
    def angles(self):
        # joint angles 1..6
        return self._angles

    @angles.setter
    def angles(self, value):
        self._angles = value
        self.solve_straight()

    def solve_straight(self):
        """ Solves straight task and updates values. The solution always exists.
        Backlash angles are supposed to be equal to 0. TODO: if not"""
        pose = solve_straight_imported(np.hstack([self._angles, 0 * self._angles]))
        self._pose = pose

    def try_solve_forward(self, pose):
        """ Solves forward kinematic problem and updates values in case the solution exists.
        """

        try:
            pos_coord, quat = pose[:3], pose[3:]
            orient_matrix = myquat2rotm(quat)
            all_solutions_deg, _, _, _ = solve_forward(pos_coord, orient_matrix)
            # TODO: concrete selection of angles solution
            angles_value = all_solutions_deg[0, :]  # choose the first solution
            self._angles = angles_value
            self._pose = pose
            self.solved = True
        except Exception as ex:
            if ex != 'maximum recursion depth exceeded':
                pose[:3] = pose[:3] * 1000
                error_dialog = QtWidgets.QMessageBox()
                error_dialog.setWindowTitle("Ошибка")
                error_dialog.setText(f"Точка ({', '.join(list(map(str, pose)))}) недостижима!")
                error_dialog.setIcon(QtWidgets.QMessageBox.Warning)
                error_dialog.exec_()
