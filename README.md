# Planning and Control Algorithms for Robotics
PnC is a C++ library designed for generating trajectories for a robot system
and stabilizing the system over the trajectories.

## Featured Algorithms

### Trajectory Planning
- Direct Transcription
- Direct Collocation
- Constrained Direct Collocation

### Stabilizing Controller
- Linear Quadratic Regulator
- Whole Body Controller
- Inverse Kinematics

## Run the Code

### Clone the repository
```
$ git pull --recurse https://github.com/junhyeokahn/PnC.git
```

### Install Dependancies
- [dart 6.5.0](https://github.com/junhyeokahn/dart)
- [drake](https://github.com/junhyeokahn/drake)
- [gurobi](http://www.gurobi.com/)
- [mosek](https://www.mosek.com/)
- [snopt](http://ccom.ucsd.edu/~optimizers)

### Build
```
$ mkdir build && cd build && cmake..
$ make -j
$ ./cart_pole
```
