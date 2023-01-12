#ifndef _ros_odom_utils_h
#define _ros_odom_utils_h
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Odometry.h>

// Toolbox for encoding and decoding StandardTrajectories into ROS messages, and possibly other features

class odom_utils {
private:
bool read;
nav_msgs::Odometry current_heading;

public:
void outputListiner(const nav_msgs::Odometry &msg); 
bool enable_write = true;
bool enable_time_samp = false;
double now;
bool getCurrOdom(nav_msgs::Odometry * curr_heading);
};

#endif
