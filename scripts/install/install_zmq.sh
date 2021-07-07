#! /bin/bash

PATH_PACKAGE=$(pwd)

echo '# ==============================================================='
echo '# installing zmq'
echo '# ==============================================================='

if [ "$(uname)" == "Darwin" ]; then

    cd $PATH_PACKAGE/../ &&
    git clone git@github.com:zeromq/libzmq.git &&
    cd libzmq &&
    mkdir build &&
    cd build &&
    cmake .. &&
    make -j4 &&
    sudo make install &&

    cd $PATH_PACKAGE/../ &&
    git clone git@github.com:zeromq/cppzmq.git &&
    cd cppzmq &&
    mkdir build &&
    cd build &&
    cmake .. &&
    make -j4 &&
    sudo make install

elif [ "$(expr substr $(uname -s) 1 5)" == "Linux" ]; then
    sudo apt-get install libzmq3-dev

fi

cd $PATH_PACKAGE
