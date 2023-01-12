#pragma once
#include <iostream>
#include "Waypoint.h"
#include <vector>
#include <traj_gen/traj_utils/polynomial.h>
#include <math.h>
#include <boost/thread.hpp>
#include "FOV.h"

#define PI 3.14159265

typedef struct {
    Eigen::MatrixXd a, b;
    } QP_constraint; //Ax = b where A is the optimization

typedef struct {
    Eigen::MatrixXd C;
 Eigen::VectorXd d,f; //d <= Cx <= f. Where x is the opitmization variable
} QP_ineq_const;

class TrajBase
{
protected:
	int dim=4; // The dimensions of our vertices.  
	int polyOrder = 10; //we assume for this project our polynomial order is always 10
	//Virtual functions that generate basis functions based on C++
	virtual Eigen::MatrixXd generateObjFun(int minDeriv)=0; //Generates the cost matrix 
	virtual QP_constraint genConstraint(int dimension, int numConstraint)=0; //Generates equality constraints from waypoints
	virtual QP_ineq_const genInEqConstraint( int dimension)=0; //generates inequalty constraints. 
	
	//Generate Joint Constraints one matrix that handles multidimensionality constraints only relevant for single matrix solve
	virtual QP_constraint genJointConstraint()=0; //Generates the A and B matrix
	virtual QP_ineq_const genInEqJointConstraint()=0; //generates inequalty constraints. 
	virtual Eigen::MatrixXd generateJointObjFun(int minDeriv)=0; // Helper function for the gerenate
	bool condCheck(); //Checks if the global condition is satisified 
	//A vector to indicate if the first 4 dimensions are solved previously
	std::vector<bool> traj_valid{ false, false, false,false }; 

	//A single cost vector used 
	Eigen::VectorXd costVector;
	bool useCostVector = false;
	//additional constraints in case you need them 
	std::vector<QP_ineq_const> add_ineq_constr;
	std::vector<QP_constraint> add_eq_constr;
	QP_ineq_const add_joint_ineq_constr;

	//Generates the A and B matrix for a single dimension 
	QP_constraint add_joint_eq_constr;
	//Perching preset hyperparams
	float force = 4.0;
	Eigen::Vector4d impV;
	bool constrainV = true;
	float duration = 0.1;

public:

	//Waypoints the most basic measure. Contains data up to snap inside of it
	std::vector<waypoint> vertices;
	//A list of segment times this is how long you expect to pass through each way point
	//i.e. [1,2] means you travel from waypoint 0->waypoint 1 in times [ 0,1]
	//Waypoint 1 -> waypoint 2 in time [1,3]
	
	std::vector<double> segmentTimes;
	Eigen::MatrixXd coeffSolved;
	//Basic limits 0 is pose,1is velocity, 2 is acceleration used for solving times
	Eigen::VectorXd limits;	
 	virtual ~TrajBase();
	TrajBase();
	
	//Get the dimension of your waypoints
	int getDim();
	//set polynomial order
	void setPolyOrder(int p);
	int getPolyOrder();
	bool checkSolved(); //checks if the solver returned true
	void overideSolve(); //Dangerous function converts all valid to true forcefully.

	//Set time Vector automatically
	float autogenTimeSegment(); 
	//Divides the time equally given a duration
	void equalTimeSegment(double duration); 
	//Manually set a time vector
	void setTimeSegment(std::vector<double> t);
	void setCostVector(Eigen::VectorXd vector);
	//Overide the waypoints with your new set
	void setVertices(std::vector<waypoint> w);
	//Set a full stop to the trajectory
	void setFullStop();

	//Quadrotor Specific
	//Automatically calculates the landing envelope inequality constraints for the last points.
	//The Forward Velocity refers to how fast much momentum you wish to apply to landing.
	//Gripper dependent 
	//Can either use rotation matrix or declare a pitch
	void calcPerchCond(double pitch);
	bool genInEqFOV(double replan_time, Eigen::Vector3d target, Eigen::Vector4d pose, Eigen::Vector3d accel, QP_ineq_const * constraint);
	void calcPerchCond(Eigen::Matrix4d Rot);

	//Set Constraints
	//pushes a waypoint into the list
   	 void push_back(waypoint w);
	//gets a waypoint at step t
	waypoint getWaypoint(int t);
	//gives you the number of waypoints
	int numWaypoints();
	
	//General Coefficents 
	void push_ineq_constr(QP_ineq_const constr, int d);
	void push_eq_constr(QP_constraint constr, int d);
	//Add additional joint constraints
	void push_joint_ineq_constr(QP_ineq_const constr);
	void push_joint_eq_constr(QP_constraint constr);


	//clear waypoints
	void clearCostVector();
	void clear_ineq();
	void clear_eq();
	void clear_vertices();	
	void clearAll();

	//Solve the function. This is a virtual function. IT MUST BE DECLARED IN A NEW TRAJECTORY
	//returns true/false depending on solving
	virtual Eigen::MatrixXd solve(int minDeriv) =0;
	//Evaluate Trajectory
	//Calculates the current trajectory at a specific derivative order
	virtual Eigen::VectorXd calculateCurrentPt( int Order, double time)=0;
	//Evaluates the trajectory and returns a matrix
	//Dim (number of indepdent axis of motions) x 5(pose up to snap of derivtave)
	virtual Eigen::MatrixXd evalTraj(double time)=0;
	//Returns an entire list of all trajectories
	virtual Eigen::MatrixXd calculateTrajectory(int Order, double dt)=0;
	//Returns the trajectoroy basis functions with n derivatives
	//example polynomial order 3, time = 1, derivative = 0
	//[1,1,1,1]
	//example polynomial order 3, time = 1, derivative = 1
	//[0,1,2,3]
	virtual Eigen::VectorXd basis(double time, int derivative)=0;

};

