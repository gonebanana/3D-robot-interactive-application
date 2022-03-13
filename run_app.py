""" Module for run application.
Contains classes Point and AnimationWidget. """
import sys, os
import numpy as np
from PyQt5 import QtWidgets, QtGui
from PyQt5.uic import loadUi
from matplotlib.figure import Figure
from matplotlib.animation import FuncAnimation
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas, \
    NavigationToolbar2QT as NavigationToolbar

from robot_solution.modeling.simulation import find_Trans_JointAngle_JointPos
from robot_solution.modeling.transform import myquat2rotm, myquat2eiler, myeiler2quat
from robot_solution.modeling.solution import solve_straight as solve_straight_imported, solve_forward
from robot_solution.trajectory import get_trajectory


class Point():
    """ Class that describes point, given by user."""
    def __init__(self):
        # init point value corresponds to the init position of the robot with zero joint angles.
        self._pose = None  # x, y, z, quat1, quat2, quat3, quat4
        # look angles.setter
        self.angles = np.array([0] * 6)  # joint angles 1..6

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
        """ Solves forward tasks and updates values in case the solution exists and
        sets self.solved = True, updates nothing and sets self.solved = False otherwise."""
        try:
            posCoord, quat = pose[:3], pose[3:]
            orientMatrix = myquat2rotm(quat)
            allSolutionsDeg, _, _, _ = solve_forward(posCoord, orientMatrix)
            # TODO: concrete selection of angles solution
            angles_value = allSolutionsDeg[0, :]  # choose the first solution
            self._angles = angles_value
            self._pose = pose
            self.solved = True
        except Exception as excep:
            if excep != 'maximum recursion depth exceeded':
                pose[:3] = pose[:3]*1000
                error_dialog = QtWidgets.QMessageBox()
                error_dialog.setWindowTitle("Ошибка")
                error_dialog.setText(f"Точка ({', '.join(list(map(str, pose)))}) недостижима!")
                error_dialog.setIcon(QtWidgets.QMessageBox.Warning)
                error_dialog.exec_()


class AnimationWidget(QtWidgets.QMainWindow):
    """ Class for building an application. """
    def __init__(self):
        QtWidgets.QMainWindow.__init__(self)
        # ---------------------------------------------------
        # define path variables to deal with creating exe using PyInstaller
        # find packaged location
        try:
            # PyInstaller creates a temp folder and stores path in _MEIPASS
            base_path = sys._MEIPASS
        except Exception:
            base_path = os.path.abspath(".")
        # find application run_app.exe location folder
        if getattr(sys, 'frozen', False):
            self.application_path = os.path.dirname(sys.executable)
        elif __file__:
            self.application_path = os.path.dirname(__file__)
        # path to the folder buildings
        build_path = os.sep.join([base_path, 'buildings', ''])
        # ---------------------------------------------------
        loadUi(build_path + 'design.ui', self)
        # set init params
        self.N_act = 6  # number of actuators
        self.unit_len = 1000  # m -> mm
        self.input_point = Point()  # set default point, given by user
        self.set_point_to_widget()
        self.input_point_updated = True
        self.track_angles = np.empty((0, 6))
        self.track_points = []
        self.saved_tracks = []
        self.add_canvas()
        self.state_line = None

        # add icons
        icons_folder = build_path + 'icons' + os.sep
        self.statusbar.showMessage('Готов к работе')
        self.setWindowIcon(QtGui.QIcon(icons_folder + 'diakont.png'))
        self.play_button.setIcon(QtGui.QIcon(icons_folder + 'play.png'))
        self.pause_button.setIcon(QtGui.QIcon(icons_folder + 'pause.png'))
        self.resume_button.setIcon(QtGui.QIcon(icons_folder + 'resume.png'))
        self.stop_button.setIcon(QtGui.QIcon(icons_folder + 'stop.png'))

        # define player buttons actions
        self.play_button.clicked.connect(self.on_start)
        self.pause_button.clicked.connect(self.on_pause)
        self.resume_button.clicked.connect(self.on_resume)
        self.stop_button.clicked.connect(self.on_stop)
        # set init player buttons states
        self.play_button.setEnabled(False)
        self.pause_button.setEnabled(False)
        self.resume_button.setEnabled(False)
        self.stop_button.setEnabled(False)

        # define current coordinates mode buttons actions
        self.cur_cartesian_mode_button.toggled.connect(lambda: self.cur_coord.setCurrentWidget(self.cur_cartesian))
        self.cur_joint_mode_button.toggled.connect(lambda: self.cur_coord.setCurrentWidget(self.cur_joint))
        self.cur_orient_select_box.currentIndexChanged.connect(self.change_cur_orient_unit)
        # set init current mode as Cartesian coordinates
        self.cur_orient_select_box.setCurrentIndex(1)
        self.cur_cartesian_mode_button.setChecked(True)

        # define point track coordinates mode buttons actions
        self.cartesian_mode_button.toggled.connect(self.select_cartesian_mode)
        self.joint_mode_button.toggled.connect(self.select_joint_mode)
        self.orient_select_box.currentIndexChanged.connect(self.change_orient_unit)
        # set init mode as Cartesian coordinates
        self.cartesian_mode_button.setChecked(True)
        # self.orient_select_box.setCurrentIndex(0)
        self.embed_position_changed_actions()

        # define point track buttons actions
        self.add_point_button.clicked.connect(self.add_point_to_track)
        self.clear_points_button.clicked.connect(self.clear_track)
        self.delete_point_button.clicked.connect(self.delete_point_from_track)
        self.modify_point_button.clicked.connect(self.modify_point)
        self.create_track_button.clicked.connect(self.create_track)
        self.output_points.currentRowChanged.connect(self.set_point_selected)
        # set point track widget init state
        self.output_points.setCurrentRow(0)
        self.set_point_selected()

    ''' Internal operations '''
    @property
    def input_point(self):
        return self._input_point

    @input_point.setter
    def input_point(self, value, plot=False):
        self._input_point = value

    def set_point_to_widget(self):
        """ Sets all point coordinates to the widget. """
        # update input point coordinates widget
        for j, coord in enumerate(['x', 'y', 'z']):
            eval(f'self.{coord}_value.setValue(self.input_point.pose[{j}] * self.unit_len)')
        for j in range(4):
            eval(f'self.quat{j + 1}_value.setValue(self.input_point.pose[{3 + j}])')
        eiler = myquat2eiler(self.input_point.pose[3:])
        for j in range(3):
            eval(f'self.eiler{j + 1}_value.setValue(eiler[{j}])')
        for j in range(6):
            eval(f'self.angle{j + 1}.setValue(self.input_point.angles[{j}])')

    def get_point_from_widget(self):
        """ Gets current input point coordinates and counts all others. """
        try:
            point = self.input_point
            if self.coord_widget.currentIndex() == 0:
                pose = np.empty(7)
                for j, axes in enumerate(['x', 'y', 'z']):
                    pose[j] = eval(f'self.{axes}_value.value() / self.unit_len')
                if self.orient_select_box.currentIndex() == 1:
                    eiler = np.empty(3)
                    for j in range(3):
                        eiler[j] = eval(f'self.eiler{j + 1}_value.value()')
                    quat = myeiler2quat(eiler)
                else:
                    quat = np.empty(4)
                    for j in range(4):
                        quat[j] = eval(f'self.quat{j + 1}_value.value()')
                pose[3:] = quat
                point.pose = pose
            else:
                angles = np.empty(6)
                for j in range(6):
                    angles[j] = eval(f'self.angle{j + 1}.value()')
                point.angles = angles
            self.input_point = point
            self.set_point_to_widget()
        except Exception as excep:
            print('get_point_from_widget:', excep)

    def position_changed_action(self):
        """ Function embedded to the user input point change event. """
        if self.interactive_checkBox.isChecked():
            self.get_point_from_widget()
            self.plot_state()
        self.input_point_updated = True

    def embed_position_changed_actions(self):
        """ Sets actions for widget input point coordinates changing events. """
        coord_names = [f'{axes}_value' for axes in ['x', 'y', 'z']]
        coord_names += [f'quat{j + 1}_value' for j in range(4)]
        coord_names += [f'eiler{j + 1}_value' for j in range(3)]
        coord_names += [f'angle{j + 1}' for j in range(6)]
        for name in coord_names:
            # eval(f'self.{name}.editingFinished.connect(self.position_changed_action)')
            eval(f'self.{name}.valueChanged.connect(self.position_changed_action)')

    def set_point_selected(self):
        """ Sets selected by user point coordinates to corresponding spin boxes."""
        cur_row = self.output_points.currentRow()
        if cur_row == self.output_points.count() - 1:
            self.point_action_widget.setCurrentWidget(self.add_point_widget)
            self.input_point = Point()
        else:
            self.point_action_widget.setCurrentWidget(self.modify_point_widget)
            self.input_point = self.track_points[cur_row]
        self.set_point_to_widget()

    ''' Interactive actions '''
    def select_cartesian_mode(self, enabled):
        """Sets Cartesian position by given joint angles."""
        if enabled:
            if self.input_point_updated:
                self.get_point_from_widget()
                self.set_point_to_widget()
                self.input_point_updated = False
            self.coord_widget.setCurrentWidget(self.cartesian)

    def select_joint_mode(self, enabled):
        """Sets joint position by given Cartesian coordinates."""
        if enabled:
            if self.input_point_updated:
                self.get_point_from_widget()
                self.set_point_to_widget()
                self.input_point_updated = False
            self.coord_widget.setCurrentWidget(self.joint)

    def change_cur_orient_unit(self):
        """ Sets the unit of current orientation depends on value of cur_orient_select_box object. """
        if self.cur_orient_select_box.currentIndex() == 0:
            self.cur_orient.setCurrentWidget(self.cur_eiler)
        else:
            self.cur_orient.setCurrentWidget(self.cur_quat)

    def change_orient_unit(self):
        """ Sets the unit of orientation depends on value of orient_select_box object. """
        if self.orient_select_box.currentIndex() == 1:
            self.orient.setCurrentWidget(self.eiler)
        else:
            self.orient.setCurrentWidget(self.quat)

    def add_point_to_track(self):
        """ Adds point to track. """
        self.get_point_from_widget()
        self.track_angles = np.vstack((self.track_angles, self.input_point.angles))
        self.track_points.append(self.input_point)
        # add point to track widget
        num_items = self.output_points.count()
        item = QtWidgets.QListWidgetItem()
        last_num = int(self.output_points.item(num_items - 2).text().split()[-1]) if num_items > 1 else 0
        item.setText(f"точка {last_num + 1}")
        self.output_points.insertItem(num_items - 1, item)
        self.set_point_selected()  # update values
        self.play_button.setEnabled(True)
        self.statusbar.showMessage('Точка добавлена')

    def modify_point(self):
        """ Modifies point value with values, given by user """
        self.get_point_from_widget()
        cur_row = self.output_points.currentRow()
        if self.input_point.solved:
            self.track_points[cur_row] = self.input_point
            self.track_angles[cur_row, :] = self.input_point.angles
            # set "add point" as current item
            self.output_points.setCurrentRow(self.output_points.count() - 1)
        self.statusbar.showMessage('Точка изменена')

    def delete_point_from_track(self):
        cur_row = self.output_points.currentRow()
        # delete point from track
        self.track_angles = np.delete(self.track_angles, cur_row, 0)
        del self.track_points[cur_row]
        # set "add point" item as selected
        self.output_points.setCurrentRow(self.output_points.count() - 1)
        # delete selected item and select the "add point" item
        self.output_points.takeItem(cur_row)
        if self.track_angles.shape[0] == 0:
            self.play_button.setEnabled(False)
        self.statusbar.showMessage('Точка удалена')

    def clear_track(self):
        self.track_angles = np.empty((0, 6))
        self.track_points = []
        # clear output_points widget
        for step in range(self.output_points.count() - 1):
            self.output_points.takeItem(0)
        self.output_points.setCurrentRow(0)
        self.play_button.setEnabled(False)
        self.statusbar.showMessage('Путь удалён')

    def create_track(self):
        """ Creates track by points added by user. """
        self.saved_tracks.append(self.track_angles)
        # add track to track list
        num_items = self.track_list.count()
        item = QtWidgets.QListWidgetItem()
        item.setText(self.track_name_value.toPlainText())
        self.track_list.insertItem(num_items, item)
        self.statusbar.showMessage('Путь создан')

    def on_start(self):
        """ Starts animation."""
        self.statusbar.showMessage('Подсчет траектории...')
        self.clear_axes()
        self.uxSim = get_trajectory(self.track_angles, save=self.save_to_file.isChecked(),
                                    fileIn=self.application_path + os.sep + 'traj_in.csv',
                                    fileOut=self.application_path + os.sep + 'traj_out.csv')
        self.create_plot_lines()
        # SolverStep = 0.01 s = 10 ms: interval = 10
        self.ani = FuncAnimation(
            self.canvas.figure, self.update_line, repeat=False,
            frames=self.points.shape[0], blit=False, interval=10)
        # set buttons statuses
        self.play_button.setEnabled(False)
        self.pause_button.setEnabled(True)
        self.stop_button.setEnabled(True)
        # update figure
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()
        self.statusbar.showMessage('Анимация запущена')

    def on_pause(self):
        """ Pauses animation."""
        self.ani.pause()
        self.pause_button.setEnabled(False)
        self.resume_button.setEnabled(True)
        self.statusbar.showMessage('Анимация остановлена')

    def on_resume(self):
        """ Resumes animation."""
        self.ani.resume()
        self.pause_button.setEnabled(True)
        self.resume_button.setEnabled(False)
        self.statusbar.showMessage('Анимация возобновлена')

    def on_stop(self):
        """ Stops animation and clears plot window."""
        self.ani.event_source.stop()
        self.clear_axes()
        self.state_line = []
        self.play_button.setEnabled(True)
        self.pause_button.setEnabled(False)
        self.resume_button.setEnabled(False)
        self.stop_button.setEnabled(False)
        self.set_undefined_current_position()
        self.statusbar.showMessage('Готов к работе')

    ''' Functions for animation '''
    def add_canvas(self):
        """ Builds window for plots. """
        self.figure = Figure(facecolor=(200/255, 228/255, 255/255), tight_layout=True, frameon=False)
        self.axes = self.figure.add_subplot(111, projection='3d', facecolor=(231/255, 239/255, 249/255))
        self.axes.set_xlabel('x')
        self.axes.set_ylabel('y')
        self.axes.set_zlabel('z')
        self.clear_axes()
        # add to widget
        self.canvas = FigureCanvas(self.figure)
        self.mplvl.addWidget(NavigationToolbar(self.canvas, self))
        self.mplvl.addWidget(self.canvas)

    def clear_axes(self):
        """ Clears plot window and resets axes."""
        self.axes.clear()
        # TODO: set real bounds of robot space
        xlim = (-1.7 * self.unit_len, 1.7 * self.unit_len)
        ylim = (-1.7 * self.unit_len, 1.7 * self.unit_len)
        zlim = (-0.5 * self.unit_len, 2.5 * self.unit_len)
        self.axes.set_xlim(*xlim)
        self.axes.set_ylim(*ylim)
        self.axes.set_zlim(*zlim)
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()

    def plot_state(self):
        """ Plots interactive robot position by changing coordinates."""
        try:
            initial = np.hstack((self.input_point.angles, 0*self.input_point.angles))
            _, _, JointPos = find_Trans_JointAngle_JointPos(initial)
            JointPos = np.hstack((np.zeros((3, 1)), JointPos*self.unit_len))
            x, y, z = JointPos
            if not self.state_line:
                self.clear_axes()
                self.state_line, = self.axes.plot(x, y, z, lw=2, c='#a63f41', marker='.', markersize=8)
            self.state_line.set_data(np.vstack((x, y)))
            self.state_line.set_3d_properties(z)
            self.figure.canvas.draw()
            self.figure.canvas.flush_events()
        except Exception as exc:
            # Here could be Exception "maximum recursion depth exceeded in comparison. It requires just do nothing."
            pass

    def create_plot_lines(self):
        """ Creates base plot objects. """
        # add a zero point of the center of absolute system
        self.points = np.hstack((np.zeros((self.uxSim.shape[0], 3)), self.uxSim[:, 43:64]))*self.unit_len
        self.N = self.points.shape[1] // 3
        # self.points = self.uxSim[:, 43:63] * self.unit_len
        self.x, self.y, self.z = [
            self.points[:, [i*3 + j for i in range(self.N)]] for j in [0, 1, 2]]
        # create current robot line
        self.line, = self.axes.plot(
            self.x[0, :], self.y[0, :], self.z[0, :], lw=2, c='g', marker='.',
            markersize=8, label='Текущее положение')
        # create init robot line
        self.line_init, = self.axes.plot(
            self.x[0, :], self.y[0, :], self.z[0, :], lw=2, c='b', marker='.',
            markersize=8, label='Исходное положение')
        # create lines of joints track
        self.joint_lines = np.empty(self.N, dtype=object)
        for j in range(self.N):
            self.joint_lines[j], = self.axes.plot(
                self.x[0, j], self.y[0, j], self.z[0, j], c='#6b6b6b', lw=0.5)
        self.axes.legend(loc='upper right', fontsize=8)

    def update_line(self, ind):
        """ Updates lines while animation. """
        self.ani_ind = ind
        # update robot position
        self.line.set_data(np.vstack((self.x[ind, :], self.y[ind, :])))
        self.line.set_3d_properties(self.z[ind, :])
        # update joint lines
        for j in range(self.N):
            self.joint_lines[j].set_data(np.vstack((self.x[:ind + 1, j], self.y[:ind + 1, j])))
            self.joint_lines[j].set_3d_properties(self.z[:ind + 1, j])
        # show current x, y, z
        to_str = lambda el: f"{el :.2f}".center(8)
        self.cur_pose.setText(''.join(map(to_str, self.uxSim[ind, 61:64])))
        self.cur_quat_value.setText(' '.join(map(to_str, self.uxSim[ind, 64 : 68])))
        # self.cur_eiler_value.setText(text) #TODO
        for j in range(self.N_act):
            eval(f'self.cur_angle{j+1}.setText(str({self.uxSim[ind, 37 + j] :.2f}))')
        # actiond after animation stops
        if ind == self.uxSim.shape[0] - 1:
            self.play_button.setEnabled(True)
            self.pause_button.setEnabled(False)
            self.resume_button.setEnabled(False)
            self.stop_button.setEnabled(True)
            self.state_line = []
            self.statusbar.showMessage('Готов к работе')
        return [self.line]

    def set_undefined_current_position(self):
        """ Sets undefined values of current coordinates. """
        self.cur_pose.setText('Не определено')
        self.cur_quat_value.setText('Не определено')
        self.cur_eiler_value.setText('Не определено') #TODO
        for j in range(self.N_act):
            eval(f"self.cur_angle{j + 1}.setText('-')")


if __name__ == "__main__":
    qApp = QtWidgets.QApplication(sys.argv)
    aw = AnimationWidget()
    aw.show()
    sys.exit(qApp.exec_())
