#! /bin/bash

PATH_PACKAGE="$(pwd)/.."
ROOT="$PATH_PACKAGE/../.."
TARGET_DIR="$ROOT/MyCloud/draco_test_2018_12"
FOLDER_NAME=$(date +%Y%m%d_%H_%M_%S)

mkdir -p ${TARGET_DIR}/${FOLDER_NAME}
mkdir -p ${TARGET_DIR}/${FOLDER_NAME}/Config

echo "Copying txt files..."
cp ${PATH_PACKAGE}/Config/Draco/* ${TARGET_DIR}/${FOLDER_NAME}/Config/
cp ${PATH_PACKAGE}/ExperimentData/*.txt ${TARGET_DIR}/${FOLDER_NAME}/
