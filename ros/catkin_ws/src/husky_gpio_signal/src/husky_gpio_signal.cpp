#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

class GPIOSignalNode {
public:
    
    std::string gpio_pin_;  // gpio_pin_ is now a string instead of an integer
    double buffer_time_;

    GPIOSignalNode() {
    // Get the private NodeHandle to access parameters from the launch file or YAML file
    ros::NodeHandle nh_private("~");

    // Retrieve parameters, setting default values if not found
    if (!nh_private.getParam("/husky_gpio_signal/gpio_pin", gpio_pin_)) {
        gpio_pin_ = "PBB.00";  // Default GPIO pin
    }

    if (!nh_private.getParam("/husky_gpio_signal/buffer_time", buffer_time_)) {
        buffer_time_ = 180.0;  // Default buffer time
    }

    // Verify the parameter is being correctly retrieved
    ROS_INFO("GPIO Pin: %s", gpio_pin_.c_str()); // Check the value of gpio_pin_
    ROS_INFO("Buffer time: %.2f", buffer_time_);

    const char* topic_env = std::getenv("HUSKY_NODEDROP_TOPIC");
    std::string nodedrop_topic_name = topic_env ? std::string(topic_env) : "need_new_node_drop";

    drop_alert_sub_ = nh_.subscribe(nodedrop_topic_name, 10, &GPIOSignalNode::dropAlertCallback, this);

    last_trigger_time_ = ros::Time::now() - ros::Duration(buffer_time_);  // Initialize so first drop can occur immediately

    exportGPIO();
}


    ~GPIOSignalNode() {
        unexportGPIO();
    }

    void dropAlertCallback(const std_msgs::Bool::ConstPtr& msg) {
        ros::Time current_time = ros::Time::now();

        // Always log that a drop alert was received
        ROS_INFO("Drop alert received at time: %.2f sec", current_time.toSec());

        // Check if enough time has passed before triggering GPIO
        if (msg->data) {
            if (current_time - last_trigger_time_ > ros::Duration(buffer_time_)) {
                // Activate GPIO
                setGPIOValue(true);
                ROS_INFO("GPIO %s is ON", gpio_pin_.c_str());

                // Update last trigger time
                last_trigger_time_ = current_time;

                // Keep GPIO high for a brief moment and turn it off
                ros::Duration(30.0).sleep();
                setGPIOValue(false);
                ROS_INFO("GPIO %s is OFF", gpio_pin_.c_str());
            } else {
                // Log that drop was ignored due to buffer time restriction
                ROS_WARN("Drop alert ignored! Need to wait %.2f more seconds before next drop is allowed.",
                         buffer_time_ - (current_time - last_trigger_time_).toSec());
            }
        }
    }

private:
    ros::Time last_trigger_time_;
    ros::NodeHandle nh_;
    ros::Subscriber drop_alert_sub_;
    
    void exportGPIO() {
        std::stringstream command;
        command << "echo " << gpio_pin_ << " | sudo tee /sys/class/gpio/export";
        ROS_INFO("Executing command: %s", command.str().c_str());
        system(command.str().c_str());
        setGPIODirection();
    }

    void unexportGPIO() {
        std::stringstream command;
        command << "echo " << gpio_pin_ << " | sudo tee /sys/class/gpio/unexport";
        system(command.str().c_str());
    }

    void setGPIODirection() {
        std::stringstream command;
        command << "echo out | sudo tee /sys/class/gpio/" << gpio_pin_ << "/direction";
        system(command.str().c_str());
    }

    void setGPIOValue(bool state) {
        std::stringstream command;
        command << "echo " << (state ? "1" : "0") << " | sudo tee /sys/class/gpio/" << gpio_pin_ << "/value";
        system(command.str().c_str());
    }

    
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gpio_signal_node");
    GPIOSignalNode node;
    ros::spin();
    return 0;
}

