# Get gtest and/or cmake from apt  
sudo apt-get install libgtest-dev
sudo apt-get install cmake # install cmake

# gtest may already be available and if so, we have to make it and link the libraries:
cd /usr/src/gtest
sudo cmake CMakeLists.txt
sudo make

# Copy the library files to usr/lib
sudo cp *.a /usr/lib

# Create a folder and simlink
sudo mkdir /usr/local/lib/gtest
sudo ln -s /usr/lib/libgtest.a /usr/local/lib/gtest/libgtest.a
sudo ln -s /usr/lib/libgtest_main.a /usr/local/lib/gtest/libgtest_main