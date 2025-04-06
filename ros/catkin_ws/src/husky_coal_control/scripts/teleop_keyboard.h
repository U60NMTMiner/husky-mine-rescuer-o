#ifndef __RUN_MAP_H__
#define __RUN_MAP_H__


#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <time.h>
#include <sensor_msgs/Joy.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

#define RUNMAP_DEFAULT_MAX_SPEED     0.5
#define RUNMAP_DEFAULT_MAX_STEERING  1.0477198
#define RUNMAP_DEFAULT_SPEED_SCALE   0.1

class RunMap
{
public:
    RunMap(ros::NodeHandle* nodehandle);

private:

    ros::NodeHandle    _nh;
    ros::Subscriber    _subscriber;
    ros::Publisher     _publisher;

    float _MAX_SPEED;
    float _MAX_STEERING;
    float _SPEED_SCALE;


private:
    void initializeSubscribers();
    void initializePublishers();

    void subscriberCallback(const sensor_msgs::Joy& message_holder);
};

#endif


#include "run_map.h"
   RunMap::RunMap(ros::NodeHandle* nodehandle)
        :_nh(*nodehandle),
         _MAX_SPEED( RUNMAP_DEFAULT_MAX_SPEED),
         _MAX_STEERING(RUNMAP_DEFAULT_MAX_STEERING),
         _SPEED_SCALE(RUNMAP_DEFAULT_SPEED_SCALE)
    {
        ROS_INFO("Creating RunMap");
        initializeSubscribers();
        initializePublishers();
    }

    void RunMap::initializeSubscribers()
    {
        ROS_INFO("Initializing Subscribers");
        _subscriber = _nh.subscribe("/joy",
                                    1,
                                    &RunMap::subscriberCallback,
                                    this);
    }

    void RunMap::initializePublishers()
    {
        ROS_INFO("Initializing Publishers");
        _publisher = _nh.advertise<ackermann_msgs::AckermannDriveStamped>("myproject/runmap_pub", 1, true);
    }

    void RunMap::subscriberCallback(const sensor_msgs::Joy& message_holder) {
            if (message_holder.buttons[1]){ //button 1 pressed

               //do something here to find the speed and steering. 

                float speed = xxxx;
                if (speed > _MAX_SPEED){
                    speed = _MAX_SPEED;
                }
                ROS_INFO("speed:%f", speed);

                float steering = yyyy;
                if (steering > _MAX_STEERING){
                    steering = _MAX_STEERING;
                }
                ROS_INFO("steering:%f", steering);

                ackermann_msgs::AckermannDriveStamped output_msg;
                output_msg.header.stamp = ros::Time::now();
                output_msg.header.frame_id = "";
                output_msg.drive.steering_angle = steering;
                output_msg.drive.speed = speed;

                _publisher.publish(output_msg);
            }
        }


int main(int argc, char** argv)
{
    ros::init(argc, argv, "runmap");

    ros::NodeHandle nh;

    ROS_INFO("Instantiating an object of type RunMap");
    RunMap runMap(&nh);

    ROS_INFO("Going into spin; callbacks is waiting...");
    ros::spin();

    return 0;
}
