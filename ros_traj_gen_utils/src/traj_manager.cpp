#include <Eigen/Eigen>
#include <ros/ros.h>
#include <iostream>
#include <traj_gen/trajectory/Waypoint.h>
#include <traj_gen/trajectory/QPpolyTraj.h>
#include <traj_gen/traj_utils/polynomial.h>
#include <traj_gen/trajectory/TrajBase.h>
#include <ros_traj_gen_utils/ros_traj_utils.h>
#include <ros_traj_gen_utils/ros_waypoint_utils.h>
#include <ros_traj_gen_utils/poscmd_publisher.h>
#include <ros_traj_gen_utils/ros_cuboid_utils.h>
#include <ros_traj_gen_utils/ros_replanner_utils.h>
#include <trackers_msgs/Transition.h>
#include <std_srvs/Trigger.h>

using namespace std;
ros_waypoint_utils listener;
//Publishers
ros::Publisher pubQP;
ros::Publisher visual_vel_pub_;
ros::Publisher visual_acc_pub_;

ros::Subscriber subWaypoint;
ros::Subscriber subMap;

ros::ServiceClient srv_transition_;
ros::ServiceClient hover_;
ros::Subscriber subOdomMsg;
odom_utils odomListiner;
static const std::string line_tracker_min_jerk("std_trackers/LineTrackerMinJerkAction");
static const std::string null_tracker_str("std_trackers/NullTracker");
bool useRVIZ = false;
//Subscribers
//timers 
ros::Timer timer;
//Trackers
//Replanning hyperparameters
bool replan = false;
std::string vehicle_name;
std::string odom;
waypoint_ineq_const ineq_const;
ros_cuboid_utils cube_map;
Eigen::Matrix4d target;
bool usePerch = false;
bool useVisual = false;

void init_params(){
    ros::start();
    ros::NodeHandle nh("/traj_exe");
	nh.getParam("device", vehicle_name);
	std::cout << " VEHICLE NAME " << vehicle_name <<std::endl;
    // setting up the publishers and subscribers
	ros::NodeHandle nh_;
	pubQP = nh_.advertise<nav_msgs::Path>("/"+vehicle_name+"/trackers_manager/qp_tracker/qp_trajectory_pos", 10);
	visual_vel_pub_ = nh_.advertise<nav_msgs::Path>("/"+vehicle_name+"/trackers_manager/qp_tracker/qp_trajectory_vel", 10);
	visual_acc_pub_ = nh_.advertise<nav_msgs::Path>("/"+vehicle_name+"/trackers_manager/qp_tracker/qp_trajectory_acc", 10);

	subWaypoint = nh.subscribe(vehicle_name+"/waypoints", 10, &ros_waypoint_utils::waypointListiner, &listener);
	srv_transition_ = nh.serviceClient<trackers_msgs::Transition>(vehicle_name+"/trackers_manager/transition");
	hover_	= nh.serviceClient<std_srvs::Trigger>(vehicle_name+"/mav_services/hover");
	std::string odom_frame = "/odom";
	nh.getParam("odom_frame", odom_frame);
	std::vector<double> select_target;
	//load a preselcted target
	if(nh.getParam("target_pose",select_target)){
		int count =0;
		for (int i = 0;i<4;i++){
			for (int j = 0;j<4;j++){
				target(i,j) = select_target[count];
				//Q(i,j) = Qin[count];
					count +=1;
				}
			}
			usePerch = true;
		std::cout << " WE ARE USING A TARGET " << target <<std::endl;
	}
	std::string odom_topic = vehicle_name+odom_frame;
	subOdomMsg = nh.subscribe(odom_topic, 10, &odom_utils::outputListiner, &odomListiner,ros::TransportHints().tcpNoDelay());
	subMap = nh.subscribe("/vox_blox_map/graph", 10, &ros_cuboid_utils::setListiner, &cube_map,ros::TransportHints().tcpNoDelay());
	ineq_const.derivOrder = 0;
	Eigen::Vector4d lower, upper,InEqDim;
	lower << -2.0, -2.0, 0.0, 0;
	upper << 2.0, 2.0, 2.0, 0;
	InEqDim << 1.0, 1.0, 1.0, 0;
	ineq_const.lower = lower;
	ineq_const.upper = upper;
	ineq_const.InEqDim = InEqDim;
}


//Visualization 
void visualize_paths(TrajBase * traj ){
	ros::NodeHandle nh;
	//encode and publish the msg path to see that way it should be following also velocity and acceleration
	nav_msgs::Path msgQP = ros_traj_utils::encodePath(0, traj, listener.getFrameId()) ;
	pubQP.publish(msgQP);
	//Do 2D Visualization
	bool display2D = false;
	nh.getParam("display_2D",display2D);
	if(display2D){
		msgQP = ros_traj_utils::encodePath(1,traj,"world");
		visual_vel_pub_.publish(msgQP);
		msgQP = ros_traj_utils::encodePath(2,traj,"world");
		visual_acc_pub_.publish(msgQP);
	}	
}

void executeOneShotTraj(std::vector<waypoint>  vertices, poscmd_publisher * controller, TrajBase * traj){
    ros_replan_utils replanner(traj, &odomListiner, &vertices, false);
	if(usePerch){
		std::cout << target <<std::endl;
		replanner.initialPlan(3, target);
	}
	else{
		replanner.initialPlan(4);
	}
	visualize_paths(traj);
	trackers_msgs::Transition transition_cmd;
	//Nulltracker transition
	transition_cmd.request.tracker = null_tracker_str;

	srv_transition_.call(transition_cmd);
	//poscmd transition
	controller->startFlight( traj);
	while(controller->getState() != HOVER){
		ros::spinOnce();
	}
	controller->setEND();
	std_srvs::Trigger trigger;
	hover_.call(trigger);

}

void executeReplanTraj(std::vector<waypoint>  vertices, poscmd_publisher * controller, TrajBase * traj){
	//std::cout << "number of vertices" <<vertices->size() <<std::endl;
	std::cout << "preparation initial plan " <<std::endl;
    ros_replan_utils replanner(traj, &odomListiner, &vertices, useVisual);
	if(usePerch){
		replanner.initialPlan(3, target);
	}
	else{
		replanner.initialPlan(4);
	}
	std::cout << "preparation initial plan solved " <<std::endl;
	TrajBase * traj_use = replanner.getTraj();
	visualize_paths(traj_use);
	trackers_msgs::Transition transition_cmd;
	transition_cmd.request.tracker = null_tracker_str;
	srv_transition_.call(transition_cmd);
	controller->startFlight(traj_use);
	double t0 = double(ros::Time::now().sec) + double(ros::Time::now().nsec)*1e-9 ;
	double replan_time = 0.04;
	double time_plan = 0;
	std::cout << "TIME Start Flight " << ros::Time::now() <<std::endl;
	while(controller->getState() != HOVER){
		ros::Duration(replan_time*0.1).sleep();
		//pubTarget.publish(target);
		ros::spinOnce();
		bool replan_success = false;
		double tend =  double(ros::Time::now().sec) + double(ros::Time::now().nsec)*1e-9 ;
		double t_elap = tend - t0;
		t0 = tend;
		time_plan+=t_elap;
		//Publish apriltag Detection
		if (time_plan >=replan_time){
			//std::cout << "replan start" <<std::endl;
			double replan_timer = ros::Time::now().nsec *1e-9 ;
			replan_success = replanner.replan(4, time_plan, 0.05);
			//std::cout << "replan end" <<std::endl;
			if (replan_success){
				traj_use = replanner.getTraj();
				controller->startFlight(traj_use);
			}
			double replan_timer_end =  ros::Time::now().nsec *1e-9 ;
			std::cout << "Time ELAPSED " <<replan_timer_end-replan_timer <<std::endl;	
			time_plan = 0.0;
		}
	}
	controller->setEND();
	std_srvs::Trigger trigger;
	hover_.call(trigger);
}



int main(int argc, char** argv)
{
	ros::init(argc, argv, "polynomial_trajectory_tools_ros_example");
	init_params();
        ros::NodeHandle nh;
	std::vector<waypoint> * vertices; 
	ros::Duration(1).sleep();
	QPpolyTraj qp_traj(4);
	//Sets the Time with autogenerating time
	qp_traj.limits[1] = 5;
	qp_traj.limits[2] = 10;
	//These values of 5 means that for a 1.7m distance gives around 5/3.4 or 1.5 ish time allocated.
	TrajBase * traj;
	float dt =0.01; //Handles the timer speed
	std::string cmd_topic = vehicle_name+"/position_cmd";
	poscmd_publisher controller(cmd_topic, &timer, dt);
	bool useBern = false;
	bool replan = false;
	while(ros::ok()) {
		ros::spinOnce();
		//wait till we publish waypoints with RVIZ
		if(listener.flag==1){
			vertices = listener.getTrajectory();
			listener.flag =0; //Allow another trajectory to be queud
			traj = &qp_traj;
			nh.getParam(vehicle_name+"/replan",replan);
			std::cout << " Start Execution" <<std::endl;
			if(replan){
				executeReplanTraj(*vertices, &controller, traj);
			}
			else{
				executeOneShotTraj(*vertices, &controller, traj);
			}
		}
	}
}
