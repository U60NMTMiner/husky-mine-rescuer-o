#!/bin/bash

_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source ${_dir}/builder.sh

export HUSKY_LOGITECH=1

alias start_joy="roslaunch husky_control teleop.launch"
