#include <ros_traj_gen_utils/ros_waypoint_utils.h>
#include <iostream>
using namespace std;

static const std::string low_title[4] = { "lowX", "lowY", "lowZ", "lowW" };
static const std::string up_title[4] = { "upX", "upY", "upZ", "upW" };

std::vector<waypoint>*  ros_waypoint_utils::getTrajectory(){
	if (flag){
		return &vertices;
	}
	return NULL;
}

waypoint inequalityParse(waypoint w, int pointNum, string S ){
    ros::NodeHandle nh;
    XmlRpc::XmlRpcValue ineq_list;
	Eigen::Vector4d numIneqCon = Eigen::VectorXd::Zero(4);
	double conL;
	double upL;
    if(nh.getParam(S, ineq_list)){
		for (int8_t i = 0; i < ineq_list.size(); i++) 
		{
			waypoint_ineq_const ineq_temp;
			XmlRpc::XmlRpcValue sublist = ineq_list[i];
			ineq_temp.derivOrder=sublist["derivOrder"];
			ineq_temp.timeOffset=sublist["toff"];
			Eigen::Vector4d low = Eigen::VectorXd::Zero(4);
			Eigen::Vector4d upper= Eigen::VectorXd::Zero(4);
			//X 
			for (int j = 0; j <4; j++){
				    std::string low_label = low_title[j];
					std::string up_label = up_title[j] ;
					//See if the constraint exsits 
					if(sublist.hasMember(low_label) && sublist.hasMember(up_label)){
						if (sublist[low_label].getType()==2){
							int temp = sublist[low_label];
							low(j) = static_cast<double>(temp);
						}
						else{
							low(j)  = sublist[low_label];
						}
						if (sublist[up_label].getType()==2){
							int temp = sublist[up_label];
							upper(j) = static_cast<double>(temp);
						}
						else{
							upper(j)  = sublist[up_label];
						}
						//Final check to see if the constraint makes sense i.e upper > low
						if(upper(j) < low(j)){
							std::cout << "AXIS: " << j << std::endl;
							std::cout << "error constraint: " << S << std::endl;
							std::cout << "Ignored" << std::endl;
							numIneqCon(j)=0;
						}
						else{
							numIneqCon(j)=1;
						}
					}
				}
				//std::cout << "upper Con" << upper <<std::endl;
				//std::cout << "lower Con" << low <<std::endl;
				ineq_temp.lower = low;
				ineq_temp.upper = upper;
				ineq_temp.InEqDim = numIneqCon;
				//std::cout << ineq_temp.InEqDim << std::endl;
				w.ineq_constraint.push_back(ineq_temp);
				}
		}
	return w;
	}
	

waypoint  additionalConstraint(waypoint w, int pointNum){
    ros::NodeHandle nh;
	std::vector<double> constraint;
	string S = "/trajectory_gen_demo/vel0";
	S[24] = pointNum+48;
	if (nh.getParam(S, constraint)){
		Eigen::Vector4d vel(constraint.data());
		w.setVel(vel);
	}
	S = "/trajectory_gen_demo/acc0";
	S[24] = pointNum+48;
	if (nh.getParam(S, constraint)){
		Eigen::Vector4d acc(constraint.data());
		w.setAccel(acc);
	}	
	 S = "/trajectory_gen_demo/jer0";
	S[24] = pointNum+48;
	if (nh.getParam(S, constraint)){
		Eigen::Vector4d jer(constraint.data());
		w.setJerk(jer);
		
	}	
	 S = "/trajectory_gen_demo/sna0";
	S[24] = pointNum+48;
	if (nh.getParam(S, constraint)){
		Eigen::Vector4d sna(constraint.data());
		w.setSnap(sna);
	}
	S =  "/trajectory_gen_demo/ineq0";
	S[25] = pointNum+48;
	//Push additional inequality constraints 
	w = inequalityParse(w, pointNum, S);
	return w;
}

void ros_waypoint_utils::waypointListiner(const nav_msgs::Path &msg){
	std::vector<geometry_msgs::PoseStamped> points= msg.poses;
	frame_id = msg.header.frame_id;
	vertices.clear();
	double prevYaw = 0;
	for(int i =0; i <points.size();i++){
		Eigen::Vector4d tempPoint(4);
		geometry_msgs::Pose pose = points[i].pose;
		tempPoint(0) = pose.position.x;
		tempPoint(1) = pose.position.y;
		tempPoint(2) = pose.position.z;
		Quaternion q;
		q.x = pose.orientation.x;
		q.y = pose.orientation.y;
		q.z = pose.orientation.z;
		q.w = pose.orientation.w;
		EulerAngles ang = ToEulerAngles(q);
		//Reversing the angles
		while((ang.yaw - prevYaw) > 3.1415926){
			ang.yaw = ang.yaw - 2*3.1415926;
		}
		while((ang.yaw - prevYaw) < -3.1415926) {
			ang.yaw = ang.yaw + 2*3.1415926;
		}
		tempPoint(3) = ang.yaw;
		prevYaw = ang.yaw;
		waypoint w(tempPoint);
		w = additionalConstraint(w,i);
		vertices.push_back(w);
	}
	flag = 1;
}

std::string ros_waypoint_utils::getFrameId(){
	return frame_id;
}

