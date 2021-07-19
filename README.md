# Planning and Control Algorithms for Robotics
PnC is a C++ library designed for generating trajectories for a robot system
and stabilizing the system over the trajectories.

## Installation
- Install [cmake](https://cmake.org/):<br/>
```source scripts/install/install_cmake.sh```
- Install [doxygen](https://www.doxygen.nl/index.html):<br/>
```source scripts/install/install_doxygen.sh```
- Install [zmq](https://zeromq.org/) and [cppzmq](https://github.com/zeromq/cppzmq):<br/>
```source scripts/install/install_zmq.sh```
- Install [protobuf](https://developers.google.com/protocol-buffers):<br/>
```source scripts/install/install_protobuf.sh```
- Install [plotjuggler](https://github.com/facontidavide/PlotJuggler):<br/>
```source scripts/install/install_plotjuggler.sh```
- Install [dart](http://dartsim.github.io/):<br/>
```source scripts/install/install_dart.sh```
- Install [pinocchio](https://github.com/stack-of-tasks/pinocchio):<br/>
```source scripts/install/install_pinocchio.sh```
- Install [anaconda](https://docs.anaconda.com/anaconda/install/)
- Install python dependancies:<br/>
```conda env create -f pnc.yml```
- Clone the repository:<br/>
```git clone https://github.com/junhyeokahn/PnC.git```

## Compile and Run the Code
- Initiate python env:<br/>
```conda activate pnc```
- Compile:<br/>
```mkdir build && cd build && cmake.. && make -j4 ```
- Run dart sim:<br/>
``` ./build/bin/run_atlas```
- Run pybullet sim:<br/>
```python simulator/pybullet/draco_main.py```

## API documentation
- Create doxygen:<br/>
```mkdir build && cd build && cmake.. && make view_docs ```
