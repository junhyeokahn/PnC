# Planning and Control Algorithms for Robotics
PnC is a C++ library designed for generating trajectories for a robot system
and stabilizing the system over the trajectories.

## Run the Code

### Clone the repository
```
$ cd 'your workspace' && git clone --recurse https://github.com/junhyeokahn/PnC.git
```

### Install Dependancies
- install cmake: ```source scripts/install/install_cmake.sh```
- install zmq and cppzmq: ```source scripts/install/install_zmq.sh```
- install protobuf: ```source scripts/install/install_protobuf.sh```
- install dart: ```source scripts/install/install_dart.sh```
- install pinocchio: ```source scripts/install/install_pinocchio.sh```
- install python dependancies: ```conda env create -f pnc.yml`

### Compile the Code
```
$ conda activate pnc
$ mkdir build && cd build && cmake..
$ make -j4
```

### Run the simulation
$ ./build/bin/run_atlas
$ python simulator/pybullet/draco_main.py
$ python simulator/pybullet/atlas_main.py
