#include <ros_traj_gen_utils/apriltag_utils.h>
using namespace std;

apriltag_utils::apriltag_utils(){
	Eigen::Matrix3d rot1;
	Eigen::Matrix3d rot2;
	H_TAG = Eigen::Matrix4d::Zero();
	H_TAG(0,1) = -1;
	H_TAG(1,0) = 1;
	H_TAG(2,2) = 1;
	H_TAG(3,3) = 1;
	//H_TAG(0,3) = -0.31;
	//H_TAG(1,3) = -0.02; //beforehand -0.07 Negative higher on pad positive lower seriously
	//Translation 
	H_RC = Eigen::Matrix4d::Zero();
	rot1(0,2) = 1;
	rot1(1,0) = -1;
	rot1(2,1) = -1;
	/*Rotx -10
	rot2(0,0) = 1;
	rot2(1,1) = 0.9848;
	rot2(1,2) = 0.1736483;
	rot2(2,1) = -0.1736483;
	rot2(2,2) = 0.9848;*/
	/* rotx -14 degs*/
	rot2(0,0) = 1;
	rot2(1,1) = 0.9689124;
	rot2(1,2) = 0.2474040;
	rot2(2,1) = -0.2474040;
	rot2(2,2) = 0.9689124;
	H_RC.block<3,3>(0,0) = rot1;//*rot2;
	H_RC(0,3)=0.3;//0.015 not tag simulator
	H_RC(3,3)=1;
}

void apriltag_utils::timerCallback(const ros::TimerEvent& event)
{

	static bool first_read = false;
	nav_msgs::Odometry header;
	//if buffer isn't full check if the last value has the same time stamp
	//if the time stamps are the same then remove it redundent call backs bad. 
	if (first_read){
		if(	odom_l.getCurrOdom(&header)){
			//std::cout << " Time of odom " <<std::endl;
			//std::cout << header.header.stamp << std::endl;
			int index = circle_start - 1;
			if(index < 0){
				index +=BUFFER_SIZE;
			}
			//remove redudent timer callbacks 
			if(abs(header.header.stamp.toSec() - odom_buffer[index].header.stamp.toSec()) < 0.01){
				return;
			}
		}
	}
	first_read = true;
	if(	odom_l.getCurrOdom(&header)){
		odom_buffer[circle_start] = header;
		circle_start=(circle_start+1)%BUFFER_SIZE;
		if(circle_start==circle_end){
			circle_end = (circle_end+1)%BUFFER_SIZE;
		}
	}
}

void apriltag_utils::sub_odom(std::string odom_topic){
    ros::NodeHandle nh;
	//std::cout << "SUbscibed " <<odom_topic <<std::endl;
	aprilOdomSub = nh.subscribe(odom_topic, 1, &odom_utils::outputListiner, &odom_l, ros::TransportHints().tcpNoDelay());
	timer = nh.createTimer(ros::Duration(0.01), &apriltag_utils::timerCallback,this);
}

//Takes the apriltage detection message and stores it in a perch_constraint  format
void apriltag_utils::aprilListen(const geometry_msgs::PoseArray &msg){
	//If no detections exit 
	if (msg.poses.size()==0){
		return;
	}
	//if buffer isn't full return 
	if ((circle_start+1)!=circle_end){
		return;
	}
	//Check if the Odom is being read 
	double tag_read = msg.header.stamp.toSec();
	int index = circle_start -40;
	//std::cout << "Tag read" <<std::endl;
	if(index <0){
		index+=BUFFER_SIZE;
	}
	if(tag_read > odom_buffer[index].header.stamp.toSec()){
		while(tag_read > odom_buffer[index].header.stamp.toSec()){
			//std::cout << tag_read - odom_buffer[index].header.stamp.toSec() <<std::endl;
			index=(index+1)%BUFFER_SIZE;
			if(index == circle_start){
				//flag =0;
				//std::cout << "Buffer failed 1" <<std::endl;
				return;
			}
		}
	}
	else{
		while(tag_read < odom_buffer[index].header.stamp.toSec()){
			index-=1;
			if(index <0){
				index+=BUFFER_SIZE;
			}
			if(index == circle_end){
				//flag =0;
				//std::cout << "Buffer failed 2" <<std::endl;
				return;
			}
		}
	}
	//std::cout << "Buffer SUccess" <<std::endl;
	current_heading = odom_buffer[index];
	current_target = msg;
	flag = 1;
}

bool apriltag_utils::getLanding(Eigen::Matrix4d * pointer_in){
	if(flag==1){
		ros::NodeHandle nh;
		//aprilOdomSub.shutdown();
		//std::cout << "Succesful Read" <<std::endl;
		joint_pose comb_pose;
		comb_pose.quad =  current_heading;
		comb_pose.target = current_target.poses[0];
		*pointer_in = WorldRot(comb_pose);
		/*std::cout << " Current quadrotor " <<std::endl;
		std::cout << current_heading.header.stamp <<std::endl;
		std::cout << " Current Target " <<std::endl;
		std::cout << current_target.header.stamp <<std::endl;*/
		//flag = 0;
		//std::cout << "Odometry listiner thereshold " << odom_l.now - current_target.header.stamp.toSec() <<std::endl;
		//aprilOdomSub = nh.subscribe(topic_name, 1, &odom_utils::outputListiner, &odom_l);
		return true;
	}
	return false;

} 

bool apriltag_utils::getLanding(Eigen::Matrix4d * pointer_in, nav_msgs::Odometry * msg2){
	if(flag==1){
		ros::NodeHandle nh;
		//aprilOdomSub.shutdown();
		//std::cout << "Succesful Read" <<std::endl;
		joint_pose comb_pose;
		comb_pose.quad =  current_heading;
		comb_pose.target = current_target.poses[0];
		*pointer_in = WorldRot(comb_pose);
		//msg2->header  = current_target.header;
		//std::cout << " Current difference " <<std::endl;
		//std::cout << current_heading.header.stamp -current_target.header.stamp <<std::endl;
		//std::cout << " Current Target " <<std::endl;
		//std::cout << current_target.header.stamp <<std::endl;
		//std::cout << "Odometry listiner thereshold " << odom_l.now - current_target.header.stamp.toSec() <<std::endl;
		//std::cout << current_heading.header.stamp <<std::endl;
		//std::cout << current_target.header.stamp <<std::endl;
		//aprilOdomSub = nh.subscribe(topic_name, 1, &odom_utils::outputListiner, &odom_l);
		return true;
	}
	return false;

} 

nav_msgs::Odometry apriltag_utils::convertMsg(Eigen::Matrix4d H, nav_msgs::Odometry header){
	nav_msgs::Odometry  pose;
	pose.header = header.header;
	pose.pose.pose.position.x = H(0,3);
	pose.pose.pose.position.y = H(1,3);
	pose.pose.pose.position.z = H(2,3);
	Eigen::Quaterniond q(H.block<3,3>(0,0));
	pose.pose.pose.orientation.x = q.x();
	pose.pose.pose.orientation.y = q.y();
	pose.pose.pose.orientation.z = q.z();
	pose.pose.pose.orientation.w = q.w();
	return pose;
}


bool apriltag_utils::getLanding(joint_pose * pointer_in){
	if(flag==1){
		//ros::NodeHandle nh;
		//aprilOdomSub.shutdown();
		//std::cout << "Succesful Read" <<std::endl;
		pointer_in->quad =  current_heading;
		pointer_in->target = current_target.poses[0];
		//aprilOdomSub = nh.subscribe(topic_name, 1, &odom_utils::outputListiner, &odom_l);
		return true;
	}
	return false;

} 


Eigen::Matrix4d apriltag_utils::WorldRot(joint_pose pose){
	//Convert Odometry to Homogenous Transform 
	nav_msgs::Odometry odom_read = pose.quad;
	Eigen::Matrix4d H_IR= Eigen::Matrix4d::Zero();
	H_IR(0,3) = odom_read.pose.pose.position.x;
	H_IR(1,3) = odom_read.pose.pose.position.y;
	H_IR(2,3) = odom_read.pose.pose.position.z;
	H_IR(3,3)=1;
	Eigen::Quaterniond q;
	q.x() = odom_read.pose.pose.orientation.x;
	q.y() = odom_read.pose.pose.orientation.y;
	q.z() = odom_read.pose.pose.orientation.z;
	q.w() = odom_read.pose.pose.orientation.w;   // x, y, z, w in order
	/**< quaternion -> rotation Matrix */
	H_IR.block<3,3>(0,0) = q.toRotationMatrix();
	//std::cout << "H_IR" <<H_IR <<std::endl;
	//std::cout << "camera w.r. inertial frame" << H_IR*H_RC <<std::endl;

	//Convert Apriltag to Homogenous Transform;
	Eigen::Matrix4d H_CT = Eigen::Matrix4d::Zero();	
	geometry_msgs::Pose bundle = pose.target;
	H_CT(0,3) = bundle.position.x;//*0.25;
	H_CT(1,3) = bundle.position.y;//*0.25;
	H_CT(2,3) = bundle.position.z;//*0.25;
	H_CT(3,3)=1;
	q.x() = bundle.orientation.x;
	q.y() = bundle.orientation.y;
	q.z() = bundle.orientation.z;
	q.w() = bundle.orientation.w;   // x, y, z, w in order
	Eigen::Matrix3d flipper = Eigen::Matrix3d::Zero();
	flipper(0,0) = 1;
	flipper(1,1) = -1;
	flipper(2,2) = -1;
	H_CT.block<3,3>(0,0) = q.toRotationMatrix();
	H_CT = H_CT*H_TAG;
	//H_CT.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
	//std::cout << "H_CT" <<H_CT <<std::endl;
	/*
	Eigen::Matrix4d H_CT_inv = H_CT.inverse();
	std::cout << "H_CT_inv" <<H_CT_inv <<std::endl;
	H_CT.block<3,3>(0,0) = H_CT.block<3,3>(0,0) *flipper;
	std::cout << "H_CT_diag_flip" <<H_CT <<std::endl;
*/
	//std::cout << "Process Apriltag to Homogenous Matrix " <<std::endl;
	Eigen::Matrix4d H_IT = H_IR*H_RC*H_CT;
	// Simplified Perch Constraint3
	//perch_constraint land_point;
	//land_point.pos = H_IT.block<3,1>(0,3);
	//std::cout << "Position" << land_point.pos <<std::endl;
	//land_point.rot =  H_IT.block<3,1>(0,2);
	//Eigen::Vector3d ea = m.eulerAngles(2, 1, 0);
	//land_point.rot = m;
	//TEMPORARY SOLUTION SIMPLY KILL THE Y AXIS 
	H_IT(1,3) = 0;
	return H_IT;	
}
