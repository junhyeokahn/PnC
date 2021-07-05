#! /bin/bash

PATH_PACKAGE=$(pwd)

echo '# ==============================================================='
echo '# installing plotjuggler'
echo '# ==============================================================='

cd $PATH_PACKAGE/../ &&
git clone git@github.com:facontidavide/PlotJuggler.git &&
cd PlotJuggler &&
mkdir build &&
cd build &&
cmake .. &&
make -j4 &&
sudo make install

cd $PATH_PACKAGE
