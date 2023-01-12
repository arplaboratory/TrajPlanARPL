#ifndef _ros_apriltag_utils_h
#define _ros_apriltag_utils_h
#include <vector>
#include <Eigen/Eigen>
#include "odom_utils.h"
#include <ros/ros.h>
#include <ros/console.h>
//#include <ros_traj_gen_utils/AprilTagDetection.h> // message
//#include <ros_traj_gen_utils/AprilTagDetectionArray.h> // message
#include <geometry_msgs/PoseArray.h>
#include <traj_gen/traj_utils/quaternion.h>
#include <string>
// Toolbox for encoding and decoding StandardTrajectories into ROS messages, and possibly other features
const int BUFFER_SIZE = 60;

typedef struct {
    nav_msgs::Odometry quad;
    geometry_msgs::Pose target;
} joint_pose;

class apriltag_utils {


private:
//Creating the circulur buffer for odom
int circle_start = 0;
int circle_end = BUFFER_SIZE-1;
nav_msgs::Odometry odom_buffer[BUFFER_SIZE];
ros::Timer timer;
odom_utils odom_l;
ros::Subscriber aprilOdomSub;
nav_msgs::Odometry current_heading;
//PREVIOUS POSE ARRAY
geometry_msgs::PoseArray current_target;
Eigen::Matrix4d H_RC;
Eigen::Matrix4d H_TAG;
//Timer Function to stuff the cricle buffer

public:
apriltag_utils();
int flag = 0;

//Takes a stdTrajectory MSG and stores it in the private trajectory variable
void aprilListen(const geometry_msgs::PoseArray &msg); 
void syncCallback(const geometry_msgs::PoseArray &msg1,const nav_msgs::Odometry &msg2);
bool getLanding(joint_pose * pointer_in); //returns false if failed to detect perch
bool getLanding(Eigen::Matrix4d * pointer_in); //returns false if failed to detect perch
bool getLanding(Eigen::Matrix4d * pointer_in, nav_msgs::Odometry * msg2); //returns false if failed to detect perch
void sub_odom(std::string topic_name);
Eigen::Matrix4d WorldRot(joint_pose pose);
//Convert Homogenous transform to odometry for publishing
static nav_msgs::Odometry convertMsg(Eigen::Matrix4d H, nav_msgs::Odometry header);
void timerCallback(const ros::TimerEvent& event);
};

#endif
