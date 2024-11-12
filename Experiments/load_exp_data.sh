#!/bin/bash

# Set the parent directory path and tar.gz filename
PARENT_DIR="$(pwd)"
TAR_FILE="experiments_all__20241104171452"  # Replace with your actual tar.gz filename
TAR_FILE_complete=$TAR_FILE.tar.gz

# Configurable folder names
FOLDERS=("RM_Fast" "RM_Slow" "CFS" "optimizerIncremental" "optimizerBF")  # You can modify this array to change folder names

# Clear the contents of the existing folders
for folder in "${FOLDERS[@]}"; do
    rm -rf "$folder"/*
done

# Extract the tar.gz file to a temporary directory
TEMP_DIR=$(pwd)/temp
tar -xf $TAR_FILE_complete
cd $TAR_FILE
for folder in "${FOLDERS[@]}"; do
    cp -r "$folder"/* ../$folder
done

cd ..
rm -rf $TAR_FILE

