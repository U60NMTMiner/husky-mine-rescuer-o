#!/bin/bash

_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source ${_dir}/builder.sh

#export HUSKY_DESCRIPTION=$(catkin_find --first-only --without-underlays husky_description urdf/husky_OS1-64_arch.urdf.xacro 2>/dev/null)
export HUSKY_DESCRIPTION=${_dir}/../ros/catkin_ws/src/husky_description/urdf/husky_OS1-64_arch.urdf.xacro

export HUSKY_OUSTER_ENABLED=1
export HUSKY_LASER_3D_ENABLED=1
export HUSKY_LASER_3D_PREFIX=ouster
export HUSKY_LASER_SCAN_TOPIC=/scan
export HUSKY_LASER_3D_TOPIC=/ouster/points

export HUSKY_GMAP=1
export HUSKY_CART=0

export HUSKY_RSSI_THRESHOLD=-80.0
export HUSKY_NODEDROP_IP="192.168.1.11 192.168.1.12 192.168.1.13 192.168.1.15 127.0.0.1:5000"
export HUSKY_NODEDROP_TOPIC=/husky_nodedrop_node/need_new_node_drop
export HUSKY_POLLING_RATE=1.0
export HUSKY_GPIO_PIN="PBB.00" 
export HUSKY_NODE_INTV_TIME=180.0

export HUSKY_JOY=1
export HUSKY_LOGITECH=1
export HUSKY_KEYB=0

export HUSKY_VIZ=1
export HUSKY_RVIZ_CONF=${_dir}/../rviz/g_cart_n4.rviz

alias start_joy="roslaunch husky_control teleop.launch"
alias start_rviz="rqt_graph & rviz -d $HUSKY_RVIZ_CONF"

exec "$@"





