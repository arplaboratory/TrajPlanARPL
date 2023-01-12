#include <Eigen/Eigen>
#ifndef Waypoint_H
#define Waypoint_H
#include <nav_msgs/Odometry.h>
#include <vector>


typedef struct {
    int derivOrder;
    double timeOffset;
    Eigen::Vector4d lower, upper;
    Eigen::Vector4d InEqDim; //Declares wether this constraint is active or not
} waypoint_ineq_const;

//The waypoint class contains possible inputs
//
class waypoint {
private:
	Eigen::VectorXd pos; //Position constraint
	Eigen::VectorXd vel; //velcoity constraint 
	Eigen::VectorXd accel; //aceleration constraint
	Eigen::VectorXd jerk; //aceleration constraint
	Eigen::VectorXd snap; //aceleration constraint
	Eigen::VectorXd status; //This is a variable that determines the order of the waypoint.
	int dim; // how many dimensions are our vectors. 
	//For example status 0 means only position constraint is set
public: 
	std::vector<waypoint_ineq_const> ineq_constraint;

	//Constructor if we choose not to set one of the values 
	//Initialize the waypoint from an odometry message 
	waypoint(nav_msgs::Odometry odom);
	//Null means this waypoint has no velocity or acceleration constraint.
	waypoint(Eigen::VectorXd pose);
	//For the Bezier Curves you can add just an inequalit constraint
	waypoint(waypoint_ineq_const ineq_const);
	waypoint( Eigen::VectorXd pose, Eigen::VectorXd velo);
	waypoint( Eigen::VectorXd pose, Eigen::VectorXd velo, Eigen::VectorXd accele);		
	waypoint( Eigen::VectorXd pose, Eigen::VectorXd velo, Eigen::VectorXd accele, Eigen::VectorXd jerke);		
	waypoint( Eigen::VectorXd pose, Eigen::VectorXd velo, Eigen::VectorXd accele, Eigen::VectorXd jerke, Eigen::VectorXd snape);
	// The below getters have a success or fail condition. In the case the variables were never set
	//return 0 fail. These conditions do not exist.
	//Else reeturn 1 the variables exist and we placed them in the pointer
	int getPos(Eigen::VectorXd* pose);
	int getVelo(Eigen::VectorXd* velo);
	int getAccel(Eigen::VectorXd* accelo);
	int getJerk(Eigen::VectorXd* jerke);
	int getSnap(Eigen::VectorXd* snape);
	//Get Constraint puts the value of the constraint order in to that pointer if possible
	//then returns a 1 to indicate success 
	// If not possible it does nothing to your pointer and returns a 0
	//The order of your polynomial constraint is 0 for pos 1 for velocity etc...
	int getConstraint(Eigen::VectorXd* output, int order);
	Eigen::VectorXd getStatus();
	//Sets the variable throws an exception if your dimensions do not match
	void setPos(Eigen::VectorXd input);
	void setVel(Eigen::VectorXd input);
	void setAccel(Eigen::VectorXd input);
	void setJerk(Eigen::VectorXd input);
	void setSnap(Eigen::VectorXd input);

    //for Bernstein Polynomial time is irrelevant it will treat this as the whole corridor
	void addInEqualityConstraint(waypoint_ineq_const ineq_const);
	//Give dimension
	int getDim();
	//Sets all higher order derivatves 1-4 to 0 as a constraint.
	void setFullStop();
};
#endif
