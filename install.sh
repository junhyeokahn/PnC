if [ "$(uname)" == "Darwin" ]; then
    mkdir drake_binary
    cd drake_binary
    curl -o drake.tar.gz https://drake-packages.csail.mit.edu/drake/continuous/drake-latest-mac.tar.gz
    sleep 30
    sudo tar -xvzf drake.tar.gz -C /opt
elif [ "$(expr substr $(uname -s) 1 5)" == "Linux" ]; then
    mkdir drake_binary
    cd drake_binary
    curl -O https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-xenial.tar.gz
    sleep 30
    sudo tar -xvzf drake-latest-xenial.tar.gz -C /opt
fi
