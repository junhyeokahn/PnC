# Planning and Control Algorithms for Robotics
PnC is a C++ library designed for generating trajectories for a robot system
and stabilizing the system over the trajectories.

## Run the Code

### Clone the repository
Make sure [Git Lfs](https://git-lfs.github.com/) is also installed for large file meshes.
```
$ cd 'your workspace' && git clone --recurse https://github.com/junhyeokahn/PnC.git
```

### Install Required Dependancies
- run ```source Script/install_sim.sh``` for [Dart 6.9.0](https://dartsim.github.io/install_dart_on_mac.html).

### Install Optional Dependancies
- run ```source Script/install_rl_package.sh``` for Reinforcement Learning.
- [Gurobi](http://www.gurobi.com/), [Mosek](https://www.mosek.com/), [Snopt](http://ccom.ucsd.edu/~optimizers) for Optimal control.

### Compile the Code
```
$ mkdir build && cd build && cmake..
$ make -j
$ ./bin/run_draco
```


## Python Binding for A1 Quadruped

### Compile the Code
- We compile the code using Anaconda. We first need to make a python virtual environment
```
$ cd PnC/
$ conda env create -f py37.yml
$ conda activate py37
$ mkdir build && cd build && cmake ..
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

