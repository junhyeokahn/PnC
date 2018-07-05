# Planning and Control Algorithms for Robotics
PnC is a C++ library designed for generating trajectories for a robot system
and stabilizing the system over the trajectories. It is built on mathmatical
programming in [Drake](https://github.com/junhyeokahn/drake) and multi body
dynamics algorithm in [Dart](https://github.com/junhyeokahn/dart).

## Featured Algorithms

### Mathmatical Programming
- Direct Transcription
- Direct Collocation
- [Constrained Direct Collocation](https://github.com/DAIRLab/dairlib-public)
- [Embedded Conic Solver](https://github.com/embotech/ecos) using [sparse matrix routines](http://www.suitesparse.com)

### Trajectory Planning
- [Centroid Dyanimcs Planning for Humanoid](https://arxiv.org/pdf/1709.09265.pdf)
- [DIRTREL](http://zacmanchester.github.io/docs/dirtrel-auro.pdf)

### Stabilizing Controller
- [Linear Quadratic Regulator](https://github.com/RobotLocomotion/drake)
- Whole Body Controller
- [Inverse Kinematics](https://github.com/junhyeokahn/dart)

## Run the Code

### Clone the repository
Make sure [Git Lfs](https://git-lfs.github.com/) is also installed for large file meshes.
```
$ git clone --recurse https://github.com/junhyeokahn/PnC.git
```

### Install Dependancies
- [Dart 6.5.0](https://github.com/junhyeokahn/dart)
- [Drake](https://github.com/junhyeokahn/drake)
- [Gurobi](http://www.gurobi.com/)
- [Mosek](https://www.mosek.com/)
- [Snopt](http://ccom.ucsd.edu/~optimizers)

### Build
```
$ mkdir build && cd build && cmake..
$ make -j
$ ./run_draco
```
