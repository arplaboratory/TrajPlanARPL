#include <traj_gen/trajectory/Waypoint.h>
#include <traj_gen/traj_utils/quaternion.h>

#include <iostream>
using namespace std;
	//CONSTRUCTORS
	waypoint::waypoint( Eigen::VectorXd pose) {
		dim = pose.rows();
		pos = pose;
		status = Eigen::VectorXd::Zero(5);
		status[0] = 1;
	}
	
	waypoint::waypoint( nav_msgs::Odometry odom) {
		dim = 4;
		Quaternion q;
		pos = Eigen::VectorXd::Zero(4);
		vel = Eigen::VectorXd::Zero(4);
		accel = Eigen::VectorXd::Zero(4);
		jerk = Eigen::VectorXd::Zero(4);
		snap = Eigen::VectorXd::Zero(4);
		q.w = odom.pose.pose.orientation.w;
		q.x =odom.pose.pose.orientation.x;
		q.y =odom.pose.pose.orientation.y;
		q.z =odom.pose.pose.orientation.z;
		EulerAngles euler =  ToEulerAngles(q);
		//set Position 
		pos[0] = odom.pose.pose.position.x;
		pos[1] = odom.pose.pose.position.y;
		pos[2] = odom.pose.pose.position.z;
		pos[3] = euler.yaw;
		vel[0] = odom.twist.twist.linear.x;
		vel[1] = odom.twist.twist.linear.y;
		vel[2] = odom.twist.twist.linear.z;
		status = Eigen::VectorXd::Constant(5,1);
		
	}

	waypoint::waypoint(waypoint_ineq_const ineq_const) {
		ineq_constraint.push_back(ineq_const);
		status = Eigen::VectorXd::Zero(5);
	}

	waypoint::waypoint(Eigen::VectorXd pose,  Eigen::VectorXd velo, Eigen::VectorXd accele) {
		dim = pose.rows();
		pos = pose;
		if (dim == velo.rows()){
			vel = velo;
		}
		else {
			std::cout << "An exception occurred. Pose and Velocity must have the same dimension " << '\n';
			throw 20;
		}
		if (dim == accele.rows()){
			accel = accele;
		}
		else {
			std::cout << "An exception occurred. Pose and accel must have the same dimension " << '\n';
			throw 20;
		}
		status = Eigen::VectorXd::Zero(5);
		status[0] = 1;
		status[1] = 1;
		status[2] = 1;
	}


	waypoint::waypoint(Eigen::VectorXd pose,  Eigen::VectorXd velo, Eigen::VectorXd accele, Eigen::VectorXd jerke) {
		dim = pose.rows();
		pos = pose;
		if (dim == velo.rows()){
			vel = velo;
		}
		else {
			std::cout << "An exception occurred. Pose and Velocity must have the same dimension " << '\n';
			throw 20;
		}
		if (dim == accele.rows()){
			accel = accele;
		}
		else {
			std::cout << "An exception occurred. Pose and accel must have the same dimension " << '\n';
			throw 20;
		}
		if (dim == jerke.rows()){
			jerk = jerke;
		}
		else {
			std::cout << "An exception occurred. Pose and Jerk must have the same dimension " << '\n';
			throw 20;
		}
		status = Eigen::VectorXd::Zero(5);
		status[0] = 1;
		status[1] = 1;
		status[2] = 1;
		status[3] = 1;
		}

	waypoint::waypoint(Eigen::VectorXd pose,  Eigen::VectorXd velo, Eigen::VectorXd accele, Eigen::VectorXd jerke,Eigen::VectorXd snape) {
		dim = pose.rows();
		pos = pose;
		if (dim == velo.rows()){
			vel = velo;
		}
		else {
			std::cout << "An exception occurred. Pose and Velocity must have the same dimension " << '\n';
			throw 20;
		}
		if (dim == accele.rows()){
			accel = accele;
		}
		else {
			std::cout << "An exception occurred. Pose and accel must have the same dimension " << '\n';
			throw 20;
		}
		if (dim == jerke.rows()){
			jerk = jerke;
		}
		else {
			std::cout << "An exception occurred. Pose and Jerk must have the same dimension " << '\n';
			throw 20;
		}
		if (dim == jerke.rows()){
			snap = snape;
		}
		else {
			std::cout << "An exception occurred. Pose and snap must have the same dimension " << '\n';
			throw 20;
		}
		status = Eigen::VectorXd::Zero(5);
		status[0] = 1;
		status[1] = 1;
		status[2] = 1;
		status[3] = 1;
		status[4] = 1;
		}
	//GETTERS 
	int waypoint::getPos(Eigen::VectorXd* pose) {
		if (status[0] == 1){
			*pose = pos;
			return 1;
		}
		return 0;
	}
	int waypoint::getVelo(Eigen::VectorXd* velo) {
		if (status[1] == 1){
			*velo = vel;
			return 1;
		}
		return 0;
	}
	int waypoint::getAccel(Eigen::VectorXd* accelo) {
		if (status[2] == 1) {
			*accelo = accel;
			return 1;
		}
		return 0;
	}
	int waypoint::getJerk(Eigen::VectorXd* jerke) {
		if (status[3] == 1) {
			*jerke = jerk;
			return 1;
		}
		return 0;
	}	
	int waypoint::getSnap(Eigen::VectorXd* snape) {
		if (status[4] == 1) {
			*snape = snap;
			return 1;
		}
		return 0;
	}
	Eigen::VectorXd waypoint::getStatus() {
		return status;
	}

	//Gives the constarints at order X if they exist.
	// If not instead doesn't change the pointer and returns a 0
	int waypoint::getConstraint(Eigen::VectorXd* output, int order) {
		if (status[order] == 1) {
			switch(order) {
			   case 0  :
				  *output = pos;
				  return 1; //optional
			   case 1  :
				  *output = vel;
				  return 1; //optional		
			   case 2  :
				  *output = accel;
				  return 1; //optional			  
			   case 3  :
				  *output = jerk;
				  return 1; //optional			  
			   case 4  :
				  *output = snap;
				  return 1; //optional			  
			default :
					return 0;
			  }
		}
		return 0;
	}
  // Setters
  	void waypoint::setPos(Eigen::VectorXd input) {
		if (dim == input.rows()){
			pos = input;
		}
		else {
			std::cout << "An exception occurred. Pose and new Pose must have the same dimension " << '\n';
			throw 20;
		}
		status[0] = 1;
	}
	
	void waypoint::setVel(Eigen::VectorXd input) {
		if (dim == input.rows()){
			vel = input;
		}
		else {
			std::cout << "An exception occurred. Pose and new Velocity must have the same dimension " << '\n';
			throw 20;
		}
		status[1] = 1;
	}
	
	void waypoint::setAccel(Eigen::VectorXd input) {
		if (dim == input.rows()){
			accel = input;
		}
		else {
			std::cout << "An exception occurred. Pose and new accel must have the same dimension " << '\n';
			throw 20;
		}
		status[2] = 1;
	}
	
	void waypoint::setJerk(Eigen::VectorXd input) {
		if (dim == input.rows()){
			jerk = input;
		}
		else {
			std::cout << "An exception occurred. Pose and new Jerk must have the same dimension " << '\n';
			throw 20;
		}
		status[3] = 1;
	}
	
	void waypoint::setSnap(Eigen::VectorXd input) {
		if (dim == input.rows()){
			snap = input;
		}
		else {
			std::cout << "An exception occurred. Pose and new Snap must have the same dimension " << '\n';
			throw 20;
		}
		status[4] = 1;
	}
		int waypoint::getDim() {
			return dim;
		}
		
	void waypoint::setFullStop() {
		vel = Eigen::VectorXd::Zero(dim);
		accel = Eigen::VectorXd::Zero(dim);
		jerk = Eigen::VectorXd::Zero(dim);
		snap = Eigen::VectorXd::Zero(dim);
		status[0] = 1;
		status[1] = 1;
		status[2] = 1;
		status[3] = 1;
		status[4] = 1;
	}


	void waypoint::addInEqualityConstraint(waypoint_ineq_const ineq_const){
		ineq_constraint.push_back(ineq_const);
	}
