# Planning and Control Algorithms for Robotics
PnC is a C++ library designed for generating trajectories for a robot system
and stabilizing the system over the trajectories. It provides interface to the
real hardware as well as simulation
environment([Dart](https://github.com/junhyeokahn/dart),
[Drake](https://github.com/junhyeokahn/drake)) and enables to communicate with
the reinforcement learning package(
[baseline](https://github.com/openai/baselines)).

## Run the Code

### Clone the repository
Make sure [Git Lfs](https://git-lfs.github.com/) is also installed for large file meshes.
```
$ cd 'your workspace' && git clone --recurse https://github.com/junhyeokahn/PnC.git
```

### Install Required Dependancies
- [Dart 6.5.0](https://github.com/junhyeokahn/dart)
```
$ cd 'your workspace' && git clone https://github.com/junhyeokahn/dart.git
$ cd dart && git checkout release-6.4
$ mkdir build && cd build && cmake .. && make -j
$ sudo make install
```

### Install Optional Dependancies
- [Gurobi](http://www.gurobi.com/)
- [Mosek](https://www.mosek.com/)
- [Snopt](http://ccom.ucsd.edu/~optimizers)
- [Drake](https://github.com/junhyeokahn/drake)
```
$ cd 'your workspace' && git clone https://github.com/junhyeokahn/drake.git
$ cd drake && ./setup/mac/install_prereqs.sh
$ rm -rf ../drake-build && mkdir ../drake-build && cd ../drake-build
$ cmake -DWITH_GUROBI=ON -DWITH_MOSEK=ON -DWITH_SNOPT=ON ../drake # Configure Gurobi, Mosek or Snopt if needed
$ make
```
Alternatively, you could also do
```
$ cd PnC && source ./install.sh
```

### Compile the Code
```
$ mkdir build && cd build && cmake..
$ make -j
$ ./run_draco
```
