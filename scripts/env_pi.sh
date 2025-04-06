#!/bin/bash

_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source ${_dir}/builder.sh

port=$(get_usb "FTDI")
if [[ $? -eq 0 ]]; then
    export IMU_PORT=$port
else
    unset IMU_PORT
fi

source /opt/ros/noetic/setup.bash
source /home/pi/husky/ros/catkin_ws/devel/setup.bash

exec "$@"
