#!/bin/bash

set -uo pipefail  # Removed -e to allow failures

export DEBIAN_FRONTEND='noninteractive'
_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Functions for logging
log_info()    { echo -e "\033[1;32m[INFO]\033[0m $*"; }
log_warning() { echo -e "\033[1;33m[WARNING]\033[0m $*"; }
log_error()   { echo -e "\033[1;31m[ERROR]\033[0m $*"; }

run_cmd() {
    "$@" || log_warning "Command failed: $*"
}

log_info "Adding repositories"
run_cmd sudo add-apt-repository -y ppa:borglab/gtsam-release-4.0
run_cmd sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

log_info "Installing curl"
run_cmd sudo apt-get install -y curl

log_info "Adding ROS key"
run_cmd curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

log_info "Updating apt"
run_cmd sudo apt-get update -y
run_cmd sudo apt-get upgrade -y

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

log_info "Installing ROS and dependencies"
run_cmd sudo apt-get install -y "${packages[@]}"

log_info "Initializing rosdep"
run_cmd sudo rosdep init
run_cmd rosdep update

log_info "Installing rosdep dependencies for workspace"
run_cmd rosdep install --from-paths "${_dir}/../ros/catkin_ws/src" --ignore-src --rosdistro="${ROS_DISTRO}" -y

log_info "Removing conflicting package"
run_cmd sudo apt-get remove -y ros-noetic-abseil-cpp

log_info "Installing Abseil manually"
run_cmd /bin/bash "${_dir}/../ros/catkin_ws/src/cartographer/scripts/install_abseil.sh"
