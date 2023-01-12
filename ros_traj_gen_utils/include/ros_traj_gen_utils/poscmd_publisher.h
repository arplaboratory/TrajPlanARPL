#ifndef _ros_traj_arpl_utils_h
#define _ros_traj_arpl_utils_h
#include <vector>
#include <Eigen/Eigen>
#include <iostream>
#include <ros/ros.h>
#include <traj_gen/traj_utils/quaternion.h>
#include <ros/console.h>
#include <traj_gen/trajectory/Waypoint.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <traj_gen/trajectory/TrajBase.h>

#include <string>
// Toolbox for encoding and decoding StandardTrajectories into ROS messages, and possibly other features
#define HOVER 0
#define FLIGHT 1
#define END 2
class poscmd_publisher {
private:
std::vector<quadrotor_msgs::PositionCommand>  flightTraj;
volatile int count = 0;
quadrotor_msgs::PositionCommand finalState;
ros::Publisher pubCMD;
ros::Time begin;
void setNewFlightPath(TrajBase * traj);

TrajBase * currTraj;
bool normalPoly = true;
double totalTime = 0.0;
//Should be moved to polynomial messages.
public:
int state = HOVER;
std::string frame_id="simulator";
double kx=7.4;
double kv=4.8;
std::vector<quadrotor_msgs::PositionCommand> position_cmd_history;
poscmd_publisher( std::string cmd_topic, ros::Timer * timer, double dt);
static std::vector<quadrotor_msgs::PositionCommand>  arplCMDlist(double dt, double kx, double kv, std::string frame_id, TrajBase * traj); //ARPL COMMAND SPECIFIC 

void startFlight(TrajBase * traj);

void setEND();
//Timer Callback 
void timerCallback(const ros::TimerEvent& event);

void endFlight();
int getState();
};

#endif
