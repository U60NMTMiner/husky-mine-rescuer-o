#!/bin/bash

_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

gcc -o ${_dir}/getip ${_dir}/getip.cpp

# Get active IP address
# ip_addr=$(ip route get 8.8.8.8 | awk -F"src " 'NR==1{split($2,a," ");print a[1]}')
ip_addr=10.0.1.1
husky_addr=10.0.1.1

export ROS_IP=$ip_addr
export ROS_MASTER_URI="http://$husky_addr:11311"

# USB serial Lookup function
get_usb() {
    local folder=/dev/serial/by-id
    if [[ ! -d $folder ]]; then
        echo "No serial devices found"
        return 2
    fi
    local port=$(ls $folder | grep $1)
    if [[ -z $port ]]; then
        echo "Serial device $1 not found"
        return 3
    fi
    port=$(readlink $folder/$port)
    port="/dev/$(basename $port)"
    echo "$port"
    return 0
}

# Aliases
cmake_args=(
    -DCMAKE_BUILD_TYPE=Release
    -DCMAKE_WARN_DEPRECATED=OFF
)

alias cake="catkin build --cmake-args ${cmake_args[@]}"
