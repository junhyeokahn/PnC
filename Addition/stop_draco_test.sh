#! /bin/bash

PATH_PACKAGE="$(pwd)/.."

echo 'turn off video recording'
sudo kill $(pgrep avconv)

echo 'turn off data saving'
kill $(pgrep Status_Display)

echo 'turn off controller'
kill $(pgrep roslaunch)

