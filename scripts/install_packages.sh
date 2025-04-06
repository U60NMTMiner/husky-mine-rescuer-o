#!/bin/bash

set -eou pipefail
export DEBIAN_FRONTEND='noninteractive'

_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

sudo add-apt-repository -y ppa:borglab/gtsam-release-4.0
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-get install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt-get update -y
sudo apt-get upgrade -y

packages=(
    ros-noetic-ros-base
    ros-noetic-rosconsole
    ros-noetic-roscpp

    ros-noetic-urdf
    ros-noetic-xacro
    ros-noetic-joy
    ros-noetic-controller-manager
    ros-noetic-teleop-twist-joy
    ros-noetic-teleop-twist-keyboard
    ros-noetic-twist-mux
    ros-noetic-roslint
    ros-noetic-rviz
    ros-noetic-pcl-ros
    ros-noetic-navigation
    ros-noetic-robot-localization
    ros-noetic-robot-state-publisher
    ros-noetic-cv-bridge
    ros-noetic-tf2-tools
    ros-noetic-imu-transformer
    ros-noetic-imu-filter-madgwick
    ros-noetic-velodyne
    ros-noetic-socketcan-interface

    python3-scipy
    python3-wstool
    python3-rosdep
    python3-catkin-tools
    python-is-python3

    libcurl4-openssl-dev
    libspdlog-dev
    libjsoncpp-dev
    libpcl-dev
    libpcap0.8-dev
    libgtsam-dev 
    libgtsam-unstable-dev
    libcv-bridge-dev
    ninja-build
    stow
)

sudo apt-get install -y ${packages[@]}


sudo rosdep init || true
rosdep update
rosdep install --from-paths $_dir/../ros/catkin_ws/src --ignore-src --rosdistro=noetic -y

sudo apt-get remove ros-noetic-abseil-cpp || true
/bin/bash $_dir/../ros/catkin_ws/src/cartographer/scripts/install_abseil.sh || true
