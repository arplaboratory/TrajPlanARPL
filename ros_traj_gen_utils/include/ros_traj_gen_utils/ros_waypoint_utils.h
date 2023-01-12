#ifndef _ros_waypoint_utils_h
#define _ros_waypoint_utils_h
#include <vector>
#include <Eigen/Eigen>

#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <traj_gen/trajectory/Waypoint.h>
#include <ros_traj_gen_utils/ros_traj_utils.h>
#include <traj_gen/traj_utils/quaternion.h>
#include <string>
// Toolbox for encoding and decoding StandardTrajectories into ROS messages, and possibly other features

class ros_waypoint_utils {
private:
std::vector<waypoint> vertices;
std::string frame_id = "simulator";
public:

int flag = 0;
void waypointListiner(const nav_msgs::Path &msg); //Takes a stdTrajectory MSG and stores it in the private trajectory variable
std::vector<waypoint>* getTrajectory(); //returns null if nothing is there 
std::string getFrameId();

};

#endif
