#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <husky_msgs/HuskyStatus.h>
#include <can/can.hpp>
#include <string>

int drawer_btn;
int node_btn;

bool last_drawer = false;
bool drawer_state = false;
bool node_state = false;
bool last_node = false;
bool last_estop = false;

std::string interface;

void joy_callback(const sensor_msgs::Joy::ConstPtr& joy);
void estop_callback(const husky_msgs::HuskyStatus::ConstPtr& msg);
int bash(const std::string& command);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joy_to_can");
    ros::NodeHandle nh;

    nh.param<std::string>("can_interface", interface, "can0");
    nh.param<int>("drawer_btn", drawer_btn, 1);
    nh.param<int>("node_btn", node_btn, 2);

    ros::Subscriber j_sub = nh.subscribe("joy_teleop/joy", 10, joy_callback);
    ros::Subscriber e_sub = nh.subscribe("status", 10, estop_callback);
    ros::spin();   
}

void joy_callback(const sensor_msgs::Joy::ConstPtr& joy)
{
    // Toggle drawer on button press
    if (joy->buttons[drawer_btn] && !last_drawer)
    {
        drawer_state = !drawer_state;
        if (drawer_state) {
           system("cansend can0 001#01"); 
        } else {
           system("cansend can0 001#00");
        }
        ROS_INFO("Toggling Drawer");
    }
    last_drawer = joy->buttons[drawer_btn];

    if (joy->buttons[node_btn] && !last_node)
    {
        node_state = !node_state;
        if (node_state) {
           system("cansend can0 002#01"); 
        } else {
           system("cansend can0 002#00");
        }
        ROS_INFO("Toggling node");
    }
    last_node = joy->buttons[node_btn];
}

void estop_callback(const husky_msgs::HuskyStatus::ConstPtr& status)
{
    // Send estop true constantly in case someone missed it
    // But only once when false so CAN isn't clogged
    if(status->e_stop) {
        system("cansend can0 000#01");
        ROS_INFO("Estopped");
    } else if(status->e_stop != last_estop) {
        system("cansend can0 000#00");
        ROS_INFO("Not Estopped");
    }
    last_estop = status->e_stop;
}
