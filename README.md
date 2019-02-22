# Planning and Control Algorithms for Robotics
PnC is a C++ library designed for generating trajectories for a robot system
and stabilizing the system over the trajectories.

## Examples
<img src="https://github.com/junhyeokahn/PnC/blob/master/Figures/draco_walking.gif" width="250" height="250"> <img src="https://github.com/junhyeokahn/PnC/blob/master/Figures/cart_pole.gif" width="250" height="250">

## Run the Code

### Clone the repository
Make sure [Git Lfs](https://git-lfs.github.com/) is also installed for large file meshes.
```
$ cd 'your workspace' && git clone --recurse https://github.com/junhyeokahn/PnC.git
```

### Install Required Dependancies
- [Dart 6.8.0](https://dartsim.github.io/install_dart_on_mac.html)

### Install Optional Dependancies
- run ```source Script/install_rl_package.sh``` for Reinforcement Learning
- [Gurobi](http://www.gurobi.com/), [Mosek](https://www.mosek.com/), [Snopt](http://ccom.ucsd.edu/~optimizers) for Optimal control

### Compile the Code
```
$ mkdir build && cd build && cmake..
$ make -j
$ ./bin/run_draco
```
