#! /bin/bash

PATH_PACKAGE="$(pwd)"

if [ "$(uname)" == "Darwin" ]; then
    brew install wget

    echo "# ==================================================================="
    echo "# Install TensorFlow"
    echo "# ==================================================================="
    pip install tensorflow &&

    echo "# ==================================================================="
    echo "# Install Protobuf"
    echo "# ==================================================================="
    wget -P ~/Downloads/ https://github.com/protocolbuffers/protobuf/releases/download/v3.6.0/protobuf-all-3.6.0.zip &&
    cd ~/Downloads && unzip protobuf-all-3.6.0.zip &&
    cd protobuf-3.6.0 && ./configure &&
    make -j4 &&
    sudo make install &&

    echo "# ==================================================================="
    echo "# Install zmq"
    echo "# ==================================================================="
    brew install zmq


elif [ "$(expr substr $(uname -s) 1 5)" == "Linux" ]; then
    sudo apt-get update && sudo apt-get install cmake libopenmpi-dev python3-dev zlib1g-dev wget &&

    echo "# ==================================================================="
    echo "# Install TensorFlow"
    echo "# ==================================================================="
    pip install tensorflow-gpu &&

    echo "# ==================================================================="
    echo "# Install Protobuf"
    echo "# ==================================================================="
    wget -P ~/Downloads/ https://github.com/protocolbuffers/protobuf/releases/download/v3.6.0/protobuf-all-3.6.0.tar.gz &&
    cd ~/Downloads && tar -xvzf protobuf-all-3.6.0.tar.gz &&
    cd protobuf-3.6.0 && ./configure &&
    make -j4 &&
    sudo make install &&

    echo "# ==================================================================="
    echo "# Install zmq"
    echo "# ==================================================================="

    wget -P ~/Downloads/ https://github.com/zeromq/libzmq/releases/download/v4.2.5/zeromq-4.2.5.tar.gz &&
    cd ~/Downloads && tar -xvzf zeromq-4.2.5.tar.gz &&

    sudo apt-get install -y libtool pkg-config build-essential autoconf automake uuid-dev &&

    cd zeromq-4.2.5 &&
    ./configure &&
    sudo make install &&
    sudo ldconfig &&
    ldconfig -p | grep zmq

    # Expected
    ############################################################
    # libzmq.so.5 (libc6,x86-64) => /usr/local/lib/libzmq.so.5
    # libzmq.so (libc6,x86-64) => /usr/local/lib/libzmq.so
    ############################################################
fi

echo "# ==================================================================="
echo "# Install Gym"
echo "# ==================================================================="
cd ${PATH_PACKAGE}/ReinforcementLearning/gym && pip install -e . &&

echo "# ==================================================================="
echo "# Install Baseline"
echo "# ==================================================================="
cd ${PATH_PACKAGE}/ReinforcementLearning/baselines && pip install -e . &&

cd ${PATH_PACKAGE}
