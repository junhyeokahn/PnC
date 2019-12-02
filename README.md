This branch is for DRACO walking experiment.

Installation Instructions

```
# Install Git LFS
https://git-lfs.github.com/

# Install gtest
see Script/gtest_install_hints.txt

# Clone this branch recursively
mkdir -p ~/Repository
cd ~/Repository/
git clone --recursive --branch develop/walking_experiment  https://github.com/junhyeokahn/PnC/

# Pull the large files
git lfs install
git lfs pull

# Install Dart
cd PnC
source Script/install_sim.sh

# Compile the code
cd ~/Repository/PnC
mkdir build
cd build
cmake ..
make -j4
````
