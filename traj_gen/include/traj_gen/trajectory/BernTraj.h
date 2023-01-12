#pragma once
#include "TrajBase.h"
#include <iostream>
#include "Waypoint.h"
#include <traj_gen/traj_utils/EigenQP.h>
#include <traj_gen/ooqp_interface/OoqpEigenInterface.hpp>

#include <vector>
#include <cassert>

class BernTraj : public TrajBase
{
private:
	Eigen::MatrixXd generateObjFun(int minDeriv); //Generates the Q matrix 
	Eigen::MatrixXd generateQ(int minDeriv, double time); // Helper function for the gerenate ObjFunction
	//Generates the A and B matrix for a single dimension 
	QP_constraint genConstraint(int dimension, int numConstraint); 
	QP_ineq_const genInEqConstraint( int dimension); //generates inequalty constraints. 
    Eigen::MatrixXd bezierDerivMatrix(int order,double time);
    Eigen::VectorXd QPSolver(Eigen::MatrixXd &P, const Eigen::VectorXd &q, QP_constraint qp);
    Eigen::VectorXd diff_basis(double time, int order);

    QP_constraint genJointConstraint(); //Generates the A and B matrix
	QP_ineq_const genInEqJointConstraint(); //generates inequalty constraints. 
	Eigen::MatrixXd generateJointObjFun(int minDeriv); // Helper function for the gerenate

	//Generate Joint Constraints one matrix that encapsulates all dimension
public:
    bool fast = false; //Determines if we fast solve. Fast solve is faster
    BernTraj();
    // However it comes at the loss of optimality will pick a near optimal solution
    //constructor d is how many dimensions a point has 3 if only X,y,Z 4 if X,Y,Z,yaw
    BernTraj(int d);
    BernTraj(int d, std::vector<double> time);
    // Uses OOQP to Solve average time 2mS on laptop
    // Solves a much tighter while also smooth trajectory compare to fast solve
    Eigen::MatrixXd solve(int minDeriv);
    //Fast Solve solves a similar problem to the QP problem though with a slight modification to increase speed
    //Average time 0.2mS on Laptop around 10x faster than solve above
    //Downside the trajectory will be slightly more loose and less optimal than solve
    //Left in to demonstrate fast solver
    Eigen::MatrixXd MTsolve(int minDeriv); 
    Eigen::MatrixXd fastMTSolve(int minDeriv); 
    Eigen::MatrixXd fastSolve(int minDeriv); 
    Eigen::MatrixXd SMsolve(int minDeriv);
	//Calculates the time series path for each dimension based on the derivative order given



 	void calcPerchCond(Eigen::Matrix3d Rot, double forwardV);

	//Evaluate Trajectory
	Eigen::VectorXd calculateCurrentPt( int Order, double time);
	Eigen::MatrixXd evalTraj(double time);
	Eigen::MatrixXd calculateTrajectory(int Order, double dt);
	Eigen::VectorXd basis(double time, int derivative);

};
