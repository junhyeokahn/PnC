#! /bin/bash

PATH_PACKAGE=$(pwd)

echo '# ==============================================================='
echo '# installing protobuf'
echo '# ==============================================================='

if [ "$(uname)" == "Darwin" ]; then

    brew install protobuf

elif [ "$(expr substr $(uname -s) 1 5)" == "Linux" ]; then

    sudo apt-get install autoconf automake libtool curl make g++ unzip &&
    cd $PATH_PACKAGE/../ &&
    git clone https://github.com/protocolbuffers/protobuf.git &&
    cd protobuf &&
    git submodule update --init --recursive &&
    ./autogen.sh &&
    ./configure &&
    make -j4 &&
    make check &&
    sudo make install &&
    sudo ldconfig

fi

cd $PATH_PACKAGE
