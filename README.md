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
- [Dart 6.8.0](https://dartsim.github.io/install_dart_on_mac.html)

### Install Optional Dependancies
- [TensorFlow](https://www.tensorflow.org/), [Baseline](https://stable-baselines.readthedocs.io/en/master/index.html) for Reinforcement Learning
- [Gurobi](http://www.gurobi.com/), [Mosek](https://www.mosek.com/), [Snopt](http://ccom.ucsd.edu/~optimizers) for Optimal control
- [zmq](https://github.com/junhyeokahn/libzmq) with [cppzmq](https://github.com/junhyeokahn/cppzmq) and [pyzmq](https://github.com/junhyeokahn/pyzmq)

### Compile the Code
```
$ mkdir build && cd build && cmake..
$ make -j
$ ./bin/run_draco
```
