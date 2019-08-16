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
- run ```source Script/install_sim.sh``` for [Dart 6.9.0](https://dartsim.github.io/install_dart_on_mac.html) and [pybullet](https://pybullet.org/wordpress/)

### Install Optional Dependancies
- run ```source Script/install_rl_package.sh``` for Reinforcement Learning
- [Gurobi](http://www.gurobi.com/), [Mosek](https://www.mosek.com/), [Snopt](http://ccom.ucsd.edu/~optimizers) for Optimal control

### Compile the Code
```
$ mkdir build && cd build && cmake..
$ make -j
$ ./bin/run_draco
```
