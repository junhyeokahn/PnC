# Planning and Control Algorithms for Robotics
PnC is a C++ library designed for generating trajectories for a robot system
and stabilizing the system over the trajectories. It is built on mathmatical
programming in [Drake](https://github.com/junhyeokahn/drake) and multi body
dynamics algorithm in [Dart](https://github.com/junhyeokahn/dart).

## Featured Algorithms

### Mathmatical Programming
- Sum of Squares
- Direct Transcription
- Direct Collocation
- [Constrained Direct Collocation](https://github.com/DAIRLab/dairlib-public)
- [Embedded Conic Solver](https://github.com/embotech/ecos) using [Sparse Matrix Routines](http://www.suitesparse.com)

### Trajectory Planning
- [Inverse Kinematics](https://github.com/junhyeokahn/dart)
- Centroid Dyanimcs Planning for Humanoid
- [DIRTREL](http://zacmanchester.github.io/docs/dirtrel-auro.pdf)
- [Funnel Libraries for Real-Time Robust Feedback Motion Planning](http://groups.csail.mit.edu/robotics-center/public_papers/Majumdar16.pdf)

### Stabilizing Controller
- [Linear Quadratic Regulator](https://github.com/RobotLocomotion/drake)
- Whole Body Controller

## Run the Code

### Clone the repository
Make sure [Git Lfs](https://git-lfs.github.com/) is also installed for large file meshes.
```
$ cd 'your workspace' && git clone --recurse https://github.com/junhyeokahn/PnC.git
```

### Install Required Dependancies
- [Dart 6.4.0](https://github.com/junhyeokahn/dart)
```
$ cd 'your workspace' && git clone https://github.com/junhyeokahn/dart.git
$ cd dart && git checkout release-6.4
$ mkdir build && cd build && cmake .. && make -j
$ sudo make install
```
- [Drake](https://github.com/junhyeokahn/drake)
```
$ cd 'your workspace' && git clone https://github.com/junhyeokahn/drake.git
$ cd drake && ./setup/mac/install_prereqs.sh
$ rm -rf ../drake-build && mkdir ../drake-build && cd ../drake-build
$ cmake -DWITH_GUROBI=ON -DWITH_MOSEK=ON -DWITH_SNOPT=ON ../drake # Configure Gurobi, Mosek or Snopt if needed
$ make
```

### Install Optional Dependancies
- [Gurobi](http://www.gurobi.com/)
- [Mosek](https://www.mosek.com/)
- [Snopt](http://ccom.ucsd.edu/~optimizers)

### Compile the Code
```
$ mkdir build && cd build && cmake..
$ make -j
$ ./run_draco
```
