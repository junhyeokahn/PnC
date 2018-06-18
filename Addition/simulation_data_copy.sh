#! /bin/bash
PATH_PACKAGE="/Users/junhyeok/Repository/RobotLocomotion/"

# Move experiment data to the other folder to plot data
cp $PATH_PACKAGE/ExperimentData/* $PATH_PACKAGE/ExperimentDataCheck/

# Make video
ffmpeg -framerate 50 -i $PATH_PACKAGE/ExperimentVideo/image%06d.png $PATH_PACKAGE/ExperimentDataCheck/video.mp4

# Remove every data for the next experiment
rm $PATH_PACKAGE/ExperimentData/*
rm $PATH_PACKAGE/ExperimentVideo/*

exit 0
