#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <fstream>  // Added for isJetsonDevice()

class GPIOSignalNode {
public:
    GPIOSignalNode() {
        
        const char* topic_env = std::getenv("HUSKY_NODEDROP_TOPIC");
        std::string nodedrop_topic_name = topic_env ? std::string(topic_env) : "need_new_node_drop";
        
        ros::NodeHandle nh_private("~");
        nh_private.getParam("gpio_pin", gpio_pin_);
        nh_private.getParam("buffer_time", buffer_time_);

        drop_alert_sub_ = nh_.subscribe(nodedrop_topic_name, 10, &GPIOSignalNode::dropAlertCallback, this);
        last_trigger_time_ = ros::Time::now();
    }

    void dropAlertCallback(const std_msgs::Bool::ConstPtr& msg) {
        if (msg->data && ros::Time::now() - last_trigger_time_ > ros::Duration(buffer_time_)) {
            callPythonScript("HIGH");
            ROS_INFO("GPIO %d is ON", gpio_pin_);
            last_trigger_time_ = ros::Time::now();

            ros::Duration(buffer_time_).sleep();  // Sleep for buffer_time_ second 
            callPythonScript("LOW");
            ROS_INFO("GPIO %d is OFF", gpio_pin_);
        } else if (!msg->data) {
            callPythonScript("LOW");
            ROS_INFO("GPIO %d is OFF (Alert cleared)", gpio_pin_);
        }
    }

private:
    void callPythonScript(const std::string& state) {
        std::stringstream command;
        command << "python3 ~/husky_coal_base/ros/catkin_ws/src/husky_gpio_signal/src/gpio_control.py " << gpio_pin_ << " " << state;
        if (!isJetsonDevice()) {
            ROS_WARN("Mock mode enabled: GPIO command will not execute real hardware actions.");
        }
        system(command.str().c_str());
    }

    bool isJetsonDevice() {
        return std::ifstream("/dev/gpiochip0").good();  // Check if Jetson GPIO exists
    }

    // **Fixed: Declare Member Variables**
    ros::NodeHandle nh_;
    ros::Subscriber drop_alert_sub_;
    int gpio_pin_;
    double buffer_time_;
    ros::Time last_trigger_time_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gpio_signal_node");
    GPIOSignalNode node;
    ros::spin();
    return 0;
}

