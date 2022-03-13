""" Contains tests of different implementations. Look into the __main__ section. """
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from robot_solution.modeling.transform import myquat2rotm
from robot_solution.modeling.simulation import find_Trans_JointAngle_JointPos
from robot_solution.modeling.solution import solve_straight, solve_forward
from robot_solution.trajectory import get_trajectory, generate_random_track


# robot_solution.modeling.solution.solve_straight
def test_solve_straight():
    """ Tests the straight solution. """
    print('This is test of straight solution.')
    angles = np.array([0] * 6)
    print('Angles:\n', angles)
    x0 = np.hstack((angles, 0 * angles))
    pose = solve_straight(x0)
    pose[:3] = pose[:3] * 1000  # convert m -> mm
    print('Pose: \n', pose, '\n')


# robot_solution.modeling.solution.solve_forward
def test_solve_forward():
    """ Tests the forward task solution. Variable pose could be changed"""
    print('This is test of forward solution.')
    # x, y, z, quat1, quat2, quat3, quat4
    pose = np.array([0.035, 0.2,   2.025, 0.71, 0, 0, 0.71])
    print('Initial cartesian pose:\n', pose)
    posCoord, quat = pose[:3], pose[3:]
    orientMatrix = myquat2rotm(quat)
    allSolutionsDeg, _, _, _ = solve_forward(posCoord, orientMatrix)
    # choose the first solution of forward task
    angles = allSolutionsDeg[0, :]
    print('Joint angles:', '\n', angles,'\n')


# robot_solution.modeling.simulation.find_Trans_JointAngle_JointPos
def count_jointpose_by_angles():
    """ Tests transformation of joint angles to its positions. """
    print('This is test of count joint positions by given angles.')
    # Value angles = np.array([0]*6) should correspond to the init robot position
    angles = np.array([0]*6)
    print('Angles:\n', angles)
    x0 = np.hstack((angles, 0*angles))
    # JointPos lines: x, y, z
    _, _, JointPos = find_Trans_JointAngle_JointPos(x0)
    JointPos = JointPos.T * 1000  # convert m -> mm
    for i in range(JointPos.shape[0]):
        print(f'Joint{i + 1}:', JointPos[i])
    print('\n')


# robot_solution.trajectory.generate_random_track
def test_random_track():
    """ Tests generation of random track. """
    print('This is test of random track.')
    track = generate_random_track()
    print('Random track is:\n', track, '\n')


# robot_solution.trajectory.get_trajectory
def test_get_trajectory():
    """ Tests generation of trajectory from random track. """
    print('This is test of get trajectory.')
    track = generate_random_track()
    print('Random track is:\n', track, '\n')
    fileIn, fileOut = 'traj_in_test.csv', 'traj_out_test.csv'
    uxSim = get_trajectory(track, save=True, fileIn=fileIn, fileOut=fileOut)
    # get joint positions and add zero point
    points = np.hstack((np.zeros((uxSim.shape[0], 3)), uxSim[:, 43: 64] * 1000))
    plot_velocities(input_points=np.empty(0), filename=fileIn)
    plot_robot_movement(points=points)


''' Functions for animation '''
def plot_robot_movement(points=np.empty(0), filename='traj_out.csv'):
    """ Plots robot animation by given joint points [nPoints x 3]
    or by the file traj_out.csv. """
    if not points.size:
        points = np.genfromtxt(filename, delimiter=',', skip_header=19, usecols=range(43, 64))
        # add zero point
        points = np.hstack((np.zeros((points.shape[0], 3)), points)) * 1000  # m -> mm
    N = points.shape[1] // 3
    x, y, z = [points[:, [i * 3 + j for i in range(N)]] for j in [0, 1, 2]]

    # set axes
    fig = plt.figure(2, frameon=False,  tight_layout=True)
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.set_xlim((-1700, 1700))
    ax.set_ylim((-1700, 1700))
    ax.set_zlim((-500, 2500))
    fig.suptitle('Движение 6-осного робота', fontsize=10)
    # create lines
    line_init, = ax.plot(x[0, :], y[0, :], z[0, :], lw=2, c='b', marker='.',
                         markersize=8, label='Исходное положение')
    line, = ax.plot(x[0, :], y[0, :], z[0, :], lw=2, c='g', marker='.',
                    markersize=8, label='Текущее положение')
    # create lines of joints track
    joint_lines = np.empty(N, dtype=object)
    for j in range(N):
        joint_lines[j], = ax.plot(
            x[0, j], y[0, j], z[0, j], c='#6b6b6b', lw=0.5)
    # Creating the Animation object
    # SolverStep = 0.01 s = 10 ms: interval = 10
    line_ani = animation.FuncAnimation(
        fig, func_ani, frames=points.shape[0], repeat=False,
        fargs=(x, y, z, line, joint_lines), interval=10, blit=False)
    ax.legend(loc='upper right', fontsize=8, frameon=False)
    plt.show()


def plot_velocities(input_points=np.empty(0), filename='traj_in.csv'):
    """ Plots velocities."""
    if not input_points.size:
        data = np.genfromtxt(filename, skip_header=1, delimiter=',')
        # plot velocities from input file
        plt.figure(1)
        plt.title('Профили скоростей')
        for i in range(6):
            plt.plot(data[:, 0], data[:, 13 + i])
        plt.grid()
        plt.show()


def func_ani(ind, x, y, z, line, joint_lines):
    """ Iterable animation function. """
    N = x.shape[1]
    line.set_data(np.vstack((x[ind, :], y[ind, :])))
    line.set_3d_properties(z[ind, :])
    # update joint lines
    for j in range(N):
        joint_lines[j].set_data(np.vstack((x[:ind + 1, j], y[:ind + 1, j])))
        joint_lines[j].set_3d_properties(z[:ind + 1, j])
    return [line, ]


if __name__ == '__main__':
    np.set_printoptions(precision=4, suppress=True) # output settings

    test_solve_straight()
    test_solve_forward()
    count_jointpose_by_angles()
    test_random_track()

    test_get_trajectory()
    # plot_robot_movement()