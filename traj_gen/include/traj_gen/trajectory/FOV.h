#pragma once
#include <Eigen/Eigen>
#include <math.h>
#include "TrajBase.h"
typedef struct {
    Eigen::MatrixXd Jacobian; //Position x,y,z and acceleration x,y,z
 double diff;
} fov_constr;


typedef struct {
    double left ; //Position x,y,z and acceleration x,y,z
    double right;
} fov_zero_order;


class FOV_constraint
{
private:
	const double r_h = .76732;
	//const double r_h = 1;
	const double g = 9.81;
	Eigen::Vector4d init_pos;
	Eigen::Vector3d init_acc;
	Eigen::Matrix3d R;
public:
	FOV_constraint(Eigen::Vector4d init_pos,Eigen::Vector3d init_acc);
	//returns a matrix that details the inequaltiy constraint
	fov_constr derivative_FOV(Eigen::Vector3d target);
	//returns the first order evaluation  
	fov_zero_order fov_eval(Eigen::Vector3d target);
};







