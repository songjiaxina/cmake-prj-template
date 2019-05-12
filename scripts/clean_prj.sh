#!/usr/bin/env bash
# author:songjiaxina@gmail.com


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

# clean certain folder
funCleanBasic() {
if [ ! -d $1 ];then
    echo "$1 folder not exists"
else
    echo "rm -rf $1"
    rm -rf $1
fi
}

funJumpToPrjRoot
funCleanBasic "./build"
funCleanBasic "./bin"
funCleanBasic "./lib"