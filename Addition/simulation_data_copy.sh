#! /bin/bash
PATH_PACKAGE=$(pwd)

# Move experiment data to the other folder to plot data
rm $PATH_PACKAGE/../ExperimentDataCheck/*
cp $PATH_PACKAGE/../ExperimentData/* $PATH_PACKAGE/../ExperimentDataCheck/

# Remove every data for the next experiment
rm $PATH_PACKAGE/../ExperimentData/*
#rm $PATH_PACKAGE/../ExperimentVideo/*
