#! /bin/bash

echo '# ==============================================================='
echo "# install doxygen"
echo '# ==============================================================='

if [ "$(uname)" == "Darwin" ]; then
    brew install doxygen
elif [ "$(expr substr $(uname -s) 1 5)" == "Linux" ]; then
    sudo apt-get install doxygen
else
    echo "[error] os not detected"
fi

