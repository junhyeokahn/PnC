# Planning and Control Algorithms for Robotics
PnC is a C++ library designed for generating trajectories for a robot system
and stabilizing the system over the trajectories. It utilizes mathmatical
programming in [drake](https://github.com/junhyeokahn/drake) and built on
the dynamics engine [dart](https://github.com/junhyeokahn/dart).

## Featured Algorithms

### Mathmatical Programming
- Direct Transcription
- Direct Collocation
- [Constrained Direct Collocation](https://github.com/DAIRLab/dairlib-public)

### Trajectory Planning
- [Centroid Dyanimcs Planning for Humanoid](https://hal.laas.fr/hal-01520248/document)
- [DIRTREL](http://zacmanchester.github.io/docs/dirtrel-auro.pdf)

### Stabilizing Controller
- [Linear Quadratic Regulator](https://github.com/RobotLocomotion/drake)
- Whole Body Controller
- [Inverse Kinematics](https://github.com/junhyeokahn/dart)

## Run the Code

### Clone the repository
```
$ git clone --recurse https://github.com/junhyeokahn/PnC.git
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
