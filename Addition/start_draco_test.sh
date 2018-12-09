#! /bin/bash
tmux rename draco
tmux rename-window experiment
session=draco
window=experiment


PATH_PACKAGE="$(pwd)/.."
VIDEO_NAME=$(date +%Y%m%d_%H_%M_%S)

tmux send-keys -t ${session}:${window}.1 'roslaunch DracoNodelet draco_nodelet.launch' Enter

tmux send-keys -t ${session}:${window}.2 'cd ~/Repository/PnC/Addition/DataManager/build' Enter
tmux send-keys -t ${session}:${window}.2 './Status_Display -v' Enter

sleep 30

tmux send-keys -t ${session}:${window}.3 'cd ~/Repository/PnC/ExperimentVideo' Enter
tmux send-keys -t ${session}:${window}.3 'sudo avconv -y -v quiet -f video4linux2 -s 720x480 -i /dev/video0 '
tmux send-keys -t ${session}:${window}.3 ${VIDEO_NAME}
tmux send-keys -t ${session}:${window}.3 '.mp4' Enter
