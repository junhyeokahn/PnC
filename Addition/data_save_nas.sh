#! /bin/bash

PATH_PACKAGE="$(pwd)/.."
ROOT="$PATH_PACKAGE/../.."
TARGET_DIR="$ROOT/MyCloud/Draco_Test_2018_12"
FOLDER_NAME=$(date +%Y%m%d_%H_%M_%S)

mkdir -p ${TARGET_DIR}/${FOLDER_NAME}

rsync -r --exclude=.DS_Store ${PATH_PACKAGE}/ExperimentData/* ${PATH_PACKAGE}/ExperimentDataCheck/
rsync -r --exclude=.DS_Store ${PATH_PACKAGE}/ExperimentData/* ${TARGET_DIR}/${FOLDER_NAME}/
rm -rf ${PATH_PACKAGE}/ExperimentData/*
