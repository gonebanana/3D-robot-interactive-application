# 3D-robot-interactive-application
Controlled motion of 6-axis robot with ability to solve both kinematic problems. Has an interactive interface and real-time visualisation.
(Contains only a part that has public access)

The current project devoted to modeling the kinematics of a 6-axis robot and interactive visualization of its movement through the implementation of an application with a user interface and 3D animation.

### Project files:
  - run_app.py
  - TESTS.py
  - robot_solution
  - buildings
  - mplwidget.py
  - requirements.txt
  - TODO.txt
  - README.txt

### HowTo
  To use the project, python must be installed. The necessary dependencies are presented in the file `requirements.txt`. Using it you can create a virtual environment for the project through the `virtualenv`. To do this, you need to execute the following sequence of commands in the command line (Windows):<br>
   ```
    >> pip install virtualenv (if virtualenv is not installed)
    >> cd <path to the project folder>
    >> virtualenv robot_env
    >> robot_env\Scripts\activate
    (robot_env)>> pip install -r requirements.txt
   ```
To run the application, run `run_app.py`. For example, in an activated virtual environment, run <br>
```
    (robot_env)>> python run_app.py
```
Similarly, you can run a file with tests TESTS.py .
To exit the environment, use the command <br>
```
    (robot_env)>> deactivete
```

### Files Description:
  1. run_app.py - file to launch the application.
  2. TESTS.py - module with tests, executable.
  3. robot_solution - python package implementing robot kinematics.
  4. buildings - a folder with auxiliary files for building the application.
  5. mplwidget.py - a module that embeds animation in the application window.
  6. requirements.txt - a list of dependencies for creating a virtual environment.
  7. TODO.txt - description of unrealized additions.
  8. README.txt
