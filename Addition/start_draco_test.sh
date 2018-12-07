#! /bin/bash

PATH_PACKAGE="$(pwd)/.."

roslaunch DracoNodelet DracoNodelet.launch &
sudo avconv -y -v quiet -f video4linux2 -s 720x480 -i /dev/video0 $PATH_PACKAGE/ExperimentData/video.mp4 &
./${PATH_PACKAGE}/Addition/DataManager/build/Status_Display -v &
