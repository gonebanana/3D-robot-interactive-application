# 3D-robot-interactive-application
Controlled motion of 6-axis robot with ability to solve both kinematic problems. Has an interactive interface and real-time visualisation.
(Contains only a part that has public access)

The current project devoted to modeling the kinematics of a 6-axis robot and interactive visualization of its movement through the implementation of an application with a user interface and 3D animation.

PROJECT FILES:
  - run_app.py
  - TESTS.py
  - robot_solution
  - buildings
  - mplwidget.py
  - requirements.txt
  - TODO.txt
  - README.txt

Start:
  To use the project, python must be installed on the computer. The necessary dependencies are presented in the file requirements.txt , with their help, you can create a virtual environment for the project through virtualenv. To do this, you need to execute the following sequence of commands on the command line (Windows):
    >> pip install virtualenv (if virtualenv is not installed)
    >> cd <path to the project folder>
    >> virtualenv robot_env
    >> robot_env\Scripts\activate <---- environment activation
    (robot_env)>> pip install -r requirements.txt
To run the application,  run run_app.py . For example, in an activated virtual environment, run
    (robot_env)>> python run_app.py
Similarly, you can run a file with tests TESTS.py .
To exit the environment, use the command
    (robot_env)>> deactivete <---- deactivating the environment

FILE DESCRIPTION:
  1. run_app.py - file to launch the application.
  2. TESTS.py - module with tests, executable.
  3. robot_solution - python package implementing robot kinematics;
    3.1. trajectory.py - module for creating a trajectory;
    3.2. modeling - python sub-package that implements the basic mathematics of the model;
    3.3. params - python-a sub-package containing static data;
  4. buildings - a folder with auxiliary files for building the application.
  5. mplwidget.py - a module that embeds animation in the application window.
  6. requirements.txt - a list of dependencies for creating a virtual environment.
  7. TODO.txt - description of unrealized additions.
  8. README.txt
