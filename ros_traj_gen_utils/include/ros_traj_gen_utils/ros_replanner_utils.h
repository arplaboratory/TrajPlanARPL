#ifndef _ros_replanner_utils_h
#define _ros_replanner_utils_h
#include <vector>
#include <Eigen/Eigen>
#include <iostream>
#include "odom_utils.h"
#include <ros/ros.h>
#include <ros/console.h>
#include <traj_gen/trajectory/Waypoint.h>
#include <traj_gen/trajectory/QPpolyTraj.h>
#include <ros_traj_gen_utils/apriltag_utils.h>
#include <string>
// Toolbox for encoding and decoding StandardTrajectories into ROS messages, and possibly other features


class ros_replan_utils {
private:
TrajBase * trajectory;
int initial_plan = 0;
odom_utils * odom_l;
std::vector<waypoint> future_v;
std::vector<double> segmentTimes;
Eigen::Matrix4d prevTarget;
double forwardV = 0.0;
double pitch = 0.0;
int fullStop = 0.0;
int curr_v =0;
//NEW PLAN WHAT IS A GOOD IDEA HERE
//Target updates
bool visualFeedback = false; 
bool fovEnable = false;
public:
ros_replan_utils();

//Initial Points you wish to pass through
ros_replan_utils(TrajBase * traj, odom_utils* odom,std::vector<waypoint>*  vertices, bool visual_in);
void set_params(TrajBase * traj, odom_utils* odom,std::vector<waypoint>*  vertices, bool visual_in);

//Get the trajectory 
TrajBase * getTraj();

//Initializing PLan
bool initialPlan(int degreeOpt);
bool initialPlan(int degreeOpt, Eigen::Matrix4d target);
//Elapesed time from the last replanning
//Replanning when you plan to this replanner uses the previous target acqueisiton
bool replan(int degreeOpt, double t_elap, double t_off);
//This replanner uses a new final target acquisition 
bool replan(int degreeOpt, double t_elap, double t_off, Eigen::Matrix4d Target);
void setFOVEnable(bool in);
void setTime(std::vector<double> times_in);

};
#endif
