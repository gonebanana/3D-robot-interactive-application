# 3D-robot-interactive-application

<img src="images\robot_animation.jpg" width="700" height="300"> <br>
<em>Shortcut of the animation with visualization of the initial, current states and intermediate trace. </em>

## Description
Controlled motion of 6-axis robot with the ability to solve both Direct and Reverse Kinematic Problems.
Has an interactive interface and real-time visualisation. The interface allows to create track points to simulate
the motion. Each point could be set by the Cartesian coordinates or quaternions with the possibility to set orientation
via Orientation or Eiler Angles. 
(Contains only a part that has a public access.)

### HowTo

To use the project, python must be installed. The necessary dependencies are presented in the file `requirements.txt`. Using it you can create a virtual environment for the project through the `virtualenv`. To do this, you need to execute the following sequence of commands in the command line (Windows):<br>
```bash
pip install virtualenv (if virtualenv is not installed)
cd <path to the project folder>
virtualenv robot_env
robot_env\Scripts\activate
pip install -r requirements.txt
```

To run the application, run `run_app.py`. For example, in an activated virtual environment, run <br>
```bash
python app/main.py
```

Similarly, you can run a file with tests.py .
To exit the environment, use the command <br>
``` bash
deactivete
```
