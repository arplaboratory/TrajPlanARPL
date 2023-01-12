#ifndef _ros_traj_utils_h
#define _ros_traj_utils_h
#include <vector>
#include <Eigen/Eigen>
#include <iostream>

#include <ros/ros.h>
#include <traj_gen/traj_utils/quaternion.h>
#include <ros/console.h>
#include <visualization_msgs/Marker.h> // message
#include <visualization_msgs/MarkerArray.h> // message

#include <nav_msgs/Path.h>
#include <traj_gen/trajectory/Waypoint.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <ros_traj_gen_utils/gnuplot.h>
#include <traj_gen/trajectory/TrajBase.h>
#include <traj_gen/trajectory/BernTraj.h>

#include <string>
// Toolbox for visualizing trajectories in both 2D/3D

static const std::string Deriv_title[5] = { "'Pos'", "'Vel'", "'accel'", "'jerk'" , "'snap'"};
static const std::string append[5] = { "Pos", "Vel", "accel", "jerk" , "snap"};
static std::vector<std::string> list_of_times;
class ros_traj_utils {
private:
int flag = 0;
//Should be moved to polynomial messages.
public:

static void graph2D_traj(int derivative_order, TrajBase  * traj); //
//static void graph2D_traj(int derivative_order, BernTraj traj); //
static void graph2D_traj(int derivative_order, double t_start, TrajBase * traj); //
static nav_msgs::Path encodePath(int derivOrder, TrajBase * traject ,const std::string& frame_id); //
//static nav_msgs::Path encodePath(int derivOrder, BernTraj traject ,const std::string& frame_id); //
static visualization_msgs::MarkerArray visualize(TrajBase  * traj, const std::string& frame_id);
static void graph2DB3( TrajBase * traj);
static void group_Plot();
};

#endif
