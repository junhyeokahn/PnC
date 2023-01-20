#! /bin/bash

PATH_PACKAGE=$(pwd)

echo '# ==============================================================='
echo '# installing googletest'
echo '# ==============================================================='

if [ "$(expr substr $(uname -s) 1 5)" == "Linux" ]; then

    cd $PATH_PACKAGE/../ &&
    git clone https://github.com/google/googletest.git &&
    cd googletest &&
    git checkout tags/release-1.11.0 &&
    mkdir build &&
    cd build &&
    cmake .. && make &&
    sudo make install &&
    sudo ldconfig

fi

cd $PATH_PACKAGE
