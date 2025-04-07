# Husky Mine Rescuer

## Overview

A Clearpath Robotics HUSKY UGV carrying an Ouster 3D LiDAR (OS1) deploys gmapping and move-base for exploring through remote control unknown and offline environments. Developed with the goal of carrying out mine search and rescue operations, it integrates its own communication node-dropping functionality. His name is still Carl.

Disclaimer: The package is a combination of existing open-source github packages and has a BSD-3-Clause license.

## The software-stack is using concepts and files from

- [HUSKY](https://github.com/husky/husky.git) - License: BSD-3-Clause
- [HUSKY-LIO-SAM](https://github.com/FarzadAziziZade/Husky-LIO-SAM.git) - License: Not listed  
- [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM.git) - License: BSD-3-Clause 
- [OUSTER-ROS](https://github.com/ouster-lidar/ouster-ros.git) - License: BSD-3-Clause 
- [OUSTER-DESCRIPTION](https://github.com/clearpathrobotics/ouster_description.git) - License: BSD-3-Clause
- [CARTOGRAPHER](https://github.com/cartographer-project/cartographer.git) - License: Apache-2.0
- [CARTOGRAPHER-ROS](https://github.com/cartographer-project/cartographer_ros.git) - License: Apache-2.0
- [HUSKY-MINE-RESCUER-N](https://github.com/U60NMTMiner/husky-mine-rescuer-n.git) - License: BSD-3-Clause - (note: previous version w/o move_base and node RSSI tracking)

## Requirements

- [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) or [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
- Requirements for Cartographer (see https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html#system-requirements)
- Requirements for LIO-SAM (see description of dependenceis at https://github.com/TixiaoShan/LIO-SAM.git)


## Installation steps


### 1.Define the ROS distro & your work-space

Note: defining commonly used parameters as environment parameters allows for faster implementation as well as for portability of commands. Note that you can either set new environment parameters every time you open a commands windows (will not be available if you open another commands window) or set them as permanent enviroment parameters in the ~/.bashrc file (available to all commands windows)

A) Set ROS distro
```
export ROS_DISTRO=noetic #if ros already installed, this should be set --> check with 'echo $ROS_DISTRO' or 'printenv | grep ROS_*'
```

B) Clone the git repo 
 
```
git clone https://github.com/U60NMTMiner/husky-mine-rescuer-o.git
```

C) Set workspace parameter
```
ws=~/husky-mine-rescuer-o
echo $ws  #check that the parameter exist and is set correctly
```

### 2.Install ROS dependencies & sensor descriptions/rospackages

A) Install dependencies either by executing:
```
. $ws/scripts/install_packages.sh 
```

OR

B) Install dependencies manually

B.1) ROS dependencies:
```
sudo apt-get install -y ros-$ROS_DISTRO-gazebo-*
sudo apt-get install -y ros-$ROS_DISTRO-navigation
sudo apt-get install -y ros-$ROS_DISTRO-teleop-twist-keyboard
sudo apt-get install -y ros-$ROS_DISTRO-robot-localization
sudo apt-get install -y ros-$ROS_DISTRO-robot-state-publisher 
```
B.2) LiDAR descriptions/rospackages & their additional dependencies:
```
sudo apt-get install -y ros-$ROS_DISTRO-velodyne-* # make sure ros-$ROS_DISTRO-velodyne-pcl is installed
sudo apt install -y ros-$ROS_DISTRO-pcl-ros
sudo apt install -y ros-$ROS_DISTRO-tf2-geometry-msgs
sudo apt install -y ros-$ROS_DISTRO-rviz
sudo apt-get install -y libpcap-dev
sudo apt-get install -y libspdlog-dev
sudo apt install -y         \
    build-essential         \
    libeigen3-dev           \
    libjsoncpp-dev          \
    libspdlog-dev           \
    libcurl4-openssl-dev    \
    cmake
```

B.3) Cartographer dependencies
```
sudo apt-get remove ros-noetic-abseil-cpp || true
/bin/bash $ws/ros/catkin_ws/src/cartographer/scripts/install_abseil.sh || true 
```

B.4) HUSKY packages - more information at [HUSKY](http://wiki.ros.org/Robots/Husky)

```
sudo apt-get install ros-$ROS_DISTRO-husky-* 
```

### 3. Build the downloaded packages

A) On HUSKY/Carl:
```
cd $ws/ros/catkin_ws && catkin_make_isolated --use-ninja -j8 
```

 OR (if only one specific package is desired)
```
cd $ws/ros/catkin_ws && catkin_make_isolated --use-ninja -j8 --pkg <package_name>
```
   
 OR (if a specific package is not desired)
```
cd $ws/ros/catkin_ws && catkin_make_isolated --use-ninja -j8 --ignore-pkg <package_name>
```


B) On a host-laptop (the remote control machine) only the control and description packages are necessary:
```
cd ros/catkin_ws && catkin_make_isolated --use-ninja -j8 --pkg husky_coal_control husky_coal_description
```

### 4. Additional settings after installation (do not skip)

A) Source your workspace (permanently):
```
echo "source $ws/ros/catkin_ws/devel_isolated/setup.bash" >> ~/.bashrc
```
B) Set necessary environment parameters:  
On HUSKY/Carl: 
```
echo "source $ws/scripts/env_husky.sh" >> ~/.bashrc
```
On host: 
```
echo "source $ws/scripts/env_ctrl.sh" >> ~/.bashrc
```
C) Source bashrc (on all machines)
```
source ~/.bashrc
```

D) Set environment parameters for the robot: 
Note 1: Some of these parameters are set to the default values when building the packages (through the 'env-hooks' files that some packages contain). Use 'echo' to check or use the following to set to desired values:
```
export HUSKY_DESCRIPTION=$(rospack find husky_coal_description)/urdf/husky_OS1-64_arch.urdf.xacro
#export HUSKY_GAZEBO_DESCRIPTION=${ws}/src/husky_coal_sim/husky_coal_description/urdf/husky_OS1-64_arch.urdf.xacro
```

E) Check environment parameters:
```
env | grep -e ROS_* -e HUSKY_*  
```

F) Add the husky ip to your /etc/hosts, where {husky-ip} is the ip address of the husky in the form of xxx.xxx.xxx.xxx. The ip address will change occasionally. Instead of running the following command, simply edit the /etc/hosts file to include the new ip. 

```
sudo bash -c "echo \"xxx.xxx.xxx.xxx    husky\" >> /etc/hosts"
```

G) Set ROS_MASTER_IP and ROS_IP parameters, where <husky-ip> and <host-ip> are the IPs of the husky and the host respectively and MUST be IP on the same network:
On HUSKY:
```
echo "ROS_MASTER_URI=http://<husky-ip>:11311" >> ~/.bashrc
echo "ROS_IP=<husky-ip>" >> ~/.bashrc
```

On host:
```
echo "ROS_MASTER_URI=http://husky:11311" >> ~/.bashrc
echo "ROS_IP=<host-ip>" >> ~/.bashrc 
ufw allow from husky
ufw allow to husky
```

## How to run the HUSKY robot remotely from host

### 1. Connect the host to the same wi-fi network as the HUSKY UGV (on start-up HUSKY deploys its own hotspot '<hotspot-name>') and ssh to HUSKY UGV
On host:
```
sudo nmcli connection up <hotspot-name>
ssh <user>@husky
#sudo systemctl restart ros.service #optional - for troubleshooting cases
```

### 2. (Optional) Various configurations through environment params

A) You may set the proper environment parameters to turn on/off sensors, bumpers, etc. See instructions at (https://github.com/husky/husky/tree/noetic-devel/husky_description)
```
export HUSKY_FRONT_BUMPER=1 #bumpers
export HUSKY_REAR_BUMPER=1
export HUSKY_SENSOR_ARCH_HEIGHT 510
export HUSKY_SENSOR_ARCH_OFFSET 0 -20 0
export HUSKY_LASER_3D_ENABLED=1
#export HUSKY_LASER_3D_TOPIC='ouster_cloud'
#export HUSKY_LASER_3D_HOST='XXX.XXX.XXX' #Find IP of Ouster LIDAR by connecting to it through browser or API. 
export HUSKY_LASER_3D_TOWER=1
export HUSKY_LASER_3D_PREFIX=''
export HUSKY_LASER_3D_PARENT='top_plate_link'
export HUSKY_LASER_3D_XYZ='0 0 0'
export HUSKY_LASER_3D_RPY='0 0 0'
```

B) Configure params for rviz visualization:
Default behavior of this package is that if HUSKY_RVIZ is true, rviz visualization is opened up alongside the gazebo simulation with the configuration file specified in HUSKY_RVIZ_CONF environment params. The second parameter is the name of the rviz file which must be located inside the /husky_coal_viz/rviz directory.

If users wish to change the default values, they can do so in terminal by:
```
export HUSKY_RVIZ=0
export HUSKY_RVIZ_CONF=<rviz_config_filename.rviz>
```

C) Define mapping and navigation packages:
Default navigation of this package is to use the gmapping and move-base packages from ros-$ROS_DISTRO-navigation package (launch from the husky_coal_navigation directory). Bby default the environment parameter that trigger this is set as: HUSKY_GMAP=1:
```
export HUSKY_GMAP=1
```

### 3. Launch operations on HUSKY UGV 

#### On husky through ssh
Start the base functions  
```
roslaunch husky_base base3.launch
```

This can be started with options by:
```
roslaunch husky_base base3.launch arg:=value arg:=value
```

E.g., the following arguments can be added to the above command:
- **port**: Physical usb port of the husky - defaults to environmental variable HUSKY_PORT or /dev/prolific
- **pi_addr**: IP adcress of the connected raspberry pi - defaults to environmental variable PI_ADDRESS or 10.XXX.XXX.XXX (deprecated)
- **ouster**: true/false. Whether to launch ouster nodes - defaults to true
- **velodyne**: true/false. Whether to launch velodyne nodes defaults to false
- **imu**: true/false. Whether to launch vectornav imu nodes on the raspberry pi - defaults to false (deprecated)
- **cart_map**: true/false. Whether to launch google cartographer nodes - defaults to false
- **gmap**: true/false. Whether to launch slam_gmapping nodes - defaults to true
- **rviz**: true/false. Whether to start-up rviz on husky/jetson - defaults to false
- **nodedrop**: true/false. Whether to start-up listening to the communication nodes throught API and drop new nodes based on RSSI - defaults to true

#### On host machine

Open 3 command windows (Ctrl+Alt+T):
1. Start logitech joystick for remote control:
```
roslaunch husky_coal_control teleop.launch
```
2. Start rviz for mapping and control through move_base package:
```
rviz -d $HUSKY_RVIZ_CONF
# rviz -d ~/husky-mine-rescuer-o/rviz/g_cart_n4.viz
```
3. Use third terminal to monitor rosnodes,rostopics,rosgraph etc.
```
rqt_graph &
rosrun rqt_tf_tree rqt_tf_tree &
rosnode list
```
## Screenshots (to be edited)
Add assets and screenshots 
