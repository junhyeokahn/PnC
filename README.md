# Planning and Control Algorithms for Robotics
PnC is a C++ library designed for generating trajectories for a robot system
and stabilizing the system over the trajectories.

## Run the Code

### Clone the repository
Make sure [Git Lfs](https://git-lfs.github.com/) is also installed for large file meshes.
```
$ cd 'your workspace' && git lfs clone https://github.com/junhyeokahn/PnC.git --branch a1_pybind
$ cd PnC && git submodule update --init --recursive
```

### Install Required Dependancies
- run ```source Script/install_sim.sh``` for [Dart 6.9.0](https://dartsim.github.io/install_dart_on_mac.html).

## Python Binding for A1 Quadruped

### Compile the Code
- We compile the code using Anaconda. We first need to make a python virtual environment
```
$ cd PnC/
$ conda env create -f mingyo.yml
$ conda activate mingyo
$ mkdir build && cd build && cmake .. -DPYTHON_EXECUTABLE=$(which python)
$ make -j
```
### Link the Library Files
```
$ cd build/lib
$ ln -s A1Interface.cpython-37m-x86_64-linux-gnu.so A1Interface.so
```
### Update the Python Test Script and Run
```
$ cd PnC/Bindings/PythonTest
$ vi main.py
```
Edit the file path in line 3 to match your file path
```
$ python main.py
```
The file should run, generate the pretty_constructor output of the MPC-WBIC Algorithm and then print out "Done"
### Using the Code
As in main.py, to use the code you will need to initialize 3 objects:
1) A1Interface
2) A1SensorData
3) A1Command
The header file that defines these objects and their members is located in PnC/PnC/A1PnC/A1Interface.hpp

### Sample Code
In the main.py script at Bindings/PythonTest you can see an example of how to use to code
for any simulator. Simply fill in the sensor data, getCommand from the interface, and then
set the joint torques in your simulation.

The URDF file to be used can be found in RobotModel/Robot/A1/a1_sim.urdf

1) kp_ and kd_ values
2) initial configuration (joint position)
3) controller loop rate (servo_rate)
for the low level can be found at Config/A1/SIMULATION.yaml

1) Joint limits (q, qdot, effort) 
for the A1 can be found at Config/A1/INTERFACE.yaml

All PnC specific parameters can be found in Config/A1/TEST/CONTROL_ARCHITECTURE_PARAMS.yaml
Important ones:
1) swing_duration
2) target_height
3) ramp_time (When trotting, the time for quad support phase of gait will be 2 * ramp_time
              because we ramp contact forces up for legs that were swinging and we ramp
              contact forces down before we enter swing. This gives us smooth contact
              transitions)
4) swing_height
5) kp/kd_com/base_ori/foot_pos/joint (These are the values with which the KinWBC will
                                      determine task priorities)

