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

funJumpToPrjRoot
find . -name "*.[hc]" | xargs -L 1 wc -l | awk '{print $1}' | while read num; do total=$((total+num)); echo $total; done