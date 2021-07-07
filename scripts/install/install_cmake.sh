#! /bin/bash

PATH_PACKAGE=$(pwd)

echo '# ==============================================================='
echo '# installing cmake'
echo '# ==============================================================='

if [ "$(uname)" == "Darwin" ]; then
    brew install cmake
elif [ "$(expr substr $(uname -s) 1 5)" == "Linux" ]; then
    sudo apt remove cmake &&
    hash -r &&
    cd $PATH_PACKAGE/../ &&
    git clone git@github.com:Kitware/CMake.git
    cd CMake &&
    ./bootstrap &&
    make -j4 &&
    sudo make install
else
    echo "[error] os not detected"
fi

cd $PATH_PACKAGE
