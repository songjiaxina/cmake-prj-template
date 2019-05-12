#!/usr/bin/env bash
# author:songjiaxina@gmail.com

NUM_PROC=2
BUILD_TYPE=Release

# jump to the project root folder
funJumpToPrjRoot() {
if [ ! -d "./src" ];then
    cd ..
    if [ ! -d "./src" ];then
        echo "error! Please run this script in projrct folder or the scripts folder!!!"
        exit
    fi
else
    echo ${PWD}
fi
}

# create build folder
funcreateBuildFolder() {
if [ ! -d "./build" ];then
    echo "build folder not exists"
    mkdir build
    cd build
else
    cd build
fi
}

funJumpToPrjRoot
funcreateBuildFolder
cmake -DCMAKE_BUILD_TYPE=$BUILD_TYPE ..
make -j$NUM_PROC