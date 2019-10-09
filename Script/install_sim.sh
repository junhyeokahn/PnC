#! /bin/bash
PATH_PACKAGE="$(pwd)"

if [ "$(uname)" == "Darwin" ]; then
    echo "# ==================================================================="
    echo "# Install Dart"
    echo "# ==================================================================="
    brew install dartsim --only-dependencies &&
    cd ${PATH_PACKAGE}/.. && git clone https://github.com/junhyeokahn/dart.git && cd dart && git checkout release-6.9 &&
    mkdir build && cd build && cmake .. && make -j6 && sudo make install

    cd ${PATH_PACKAGE}

elif [ "$(expr substr $(uname -s) 1 5)" == "Linux" ]; then
    echo "# ==================================================================="
    echo "# Install Dart"
    echo "# ==================================================================="
    sudo apt-get install build-essential cmake pkg-config git &&
    sudo apt-get install libeigen3-dev libassimp-dev libccd-dev libfcl-dev libboost-regex-dev libboost-system-dev &&
    sudo apt-get install libopenscenegraph-dev &&
    sudo apt-get install libbullet-dev &&
    sudo apt-get install libode-dev &&
    sudo apt-get install libtinyxml2-dev &&
    sudo apt-get install liburdfdom-dev &&
    sudo apt-get install libxi-dev libxmu-dev freeglut3-dev &&
    sudo apt-get install libopenscenegraph-dev &&
    sudo apt-get install liboctomap-dev &&
    cd ${PATH_PACKAGE}/.. && git clone https://github.com/junhyeokahn/dart.git && cd dart && git checkout release-6.9 &&
    mkdir build && cd build && cmake .. && make -j6 && sudo make install

    cd ${PATH_PACKAGE}
fi
