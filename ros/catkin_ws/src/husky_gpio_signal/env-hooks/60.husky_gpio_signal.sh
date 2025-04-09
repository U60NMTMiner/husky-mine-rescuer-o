#!/bin/bash

# Ensure user has access to the gpio group (Jetson only)
sudo usermod -aG gpio $USER
## troubleshooting: 
# a. check if group gpio exists: groups
# b. manually create group if it does not exist: sudo groupadd gpio
# c. reinstall Jetson. GPIO: sudo pip3 install --upgrade Jetson.GPIO

# Ensure the gpio_control.py script is executable
#chmod +x ${CMAKE_CURRENT_SOURCE_DIR}/src/gpio_control.py

## Set up Python environment for Jetson GPIO access
## Ensure that Python modules are available in the PYTHONPATH if required
#export PYTHONPATH=/usr/lib/python3.6/dist-packages:$PYTHONPATH  # Adjust this if necessary

## Make sure the Jetson GPIO tools and libraries are in the PATH
export PATH=$PATH:/usr/local/bin  # Add path for Jetson GPIO tools

##Grant sudo permission to the script for GPIO control
#echo "jetson ALL=(ALL) NOPASSWD: /usr/bin/tee" | sudo tee -a /etc/sudoers > /dev/null

## Ensure that necessary GPIO modules are loaded
# For example, loading the Jetson GPIO module (if necessary)
# This step may depend on the specific Jetson model and kernel version
if ! lsmod | grep -q "gpio"; then
    echo "Loading Jetson GPIO kernel module..."
    sudo modprobe gpio
fi

## Ensure that the user has access to the GPIO pins.
## The GPIO pins are usually controlled by the 'gpio' group or 'root' user.
## If your system requires, you can add the user to the 'gpio' group (replace <username> with your actual username)
if ! groups $(whoami) | grep -q "\bgpio\b"; then
    echo "Adding user $(whoami) to gpio group..."
    sudo usermod -aG gpio $(whoami)
    echo "You may need to log out and log back in to apply the changes."
fi

## Make sure the GPIO pins are accessible from the command line
## Granting permission to access GPIO pins directly (adjust paths if needed)
sudo chmod 666 /sys/class/gpio/export
sudo chmod 666 /sys/class/gpio/unexport
sudo chmod 666 /sys/class/gpio/PBB.00/*  # Allows access to GPIO pins

export HUSKY_NODE_INTV_TIME=180.0 # check also config/**.yaml 
export HUSKY_GPIO_PIN="PBB.00" 

