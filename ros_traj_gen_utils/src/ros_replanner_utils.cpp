#include <ros_traj_gen_utils/ros_replanner_utils.h>
#include <ros_traj_gen_utils/ros_traj_utils.h>
#include <iostream>
using namespace std;

ros_replan_utils::ros_replan_utils(){
	
}


ros_replan_utils::ros_replan_utils(TrajBase * traj, odom_utils* odom, std::vector<waypoint>* vertices, bool visual_in){
	set_params(traj,  odom, vertices, visual_in);
}


void ros_replan_utils::setTime(std::vector<double> times_in){
	segmentTimes.clear();
	segmentTimes=times_in;
}


void ros_replan_utils::set_params(TrajBase * traj, odom_utils* odom, std::vector<waypoint>* vertices, bool visual_in){
	trajectory = traj;
	odom_l = odom;
	future_v.clear();
	curr_v =0;
	std::vector<waypoint> clone_V(*vertices);
	future_v = clone_V;
    ros::NodeHandle nh;
	std::string tag_topic;	
	visualFeedback = visual_in;
	if(visualFeedback){	 
		fullStop = 0;
	}else{
		fullStop = 1;
	}
}

TrajBase * ros_replan_utils::getTraj(){
	return trajectory;
}

bool ros_replan_utils::initialPlan(int degreeOpt){
        //std::cout << "initial plan " <<std::endl;
	curr_v =0;
	nav_msgs::Odometry current_heading;
	if(odom_l->getCurrOdom(&current_heading)){
		waypoint start(current_heading);
		trajectory->push_back(start);
	}
	trajectory->vertices.clear();
    //std::cout << "first point pushed " <<std::endl;	
	for (int i =0;i < future_v.size();i++){
		trajectory->push_back(future_v[i]);
	}
        //std::cout << "last point added" <<std::endl;
	if(fullStop ==1){
		trajectory->setFullStop();
	}
	else{
		trajectory->calcPerchCond(prevTarget);
	}
       
	//set Time
        //std::cout << " initially generate segment time " <<std::endl; 
	trajectory->autogenTimeSegment();
	segmentTimes = trajectory->segmentTimes;

	//segmentTimes[0] += 0.6;	
	//segmentTimes[1] += 0.3;
	//trajectory.segmentTimes = segmentTimes;
	int count = 0;
	Eigen::MatrixXd coeffQP =  trajectory->solve(degreeOpt);
	while (!(trajectory->checkSolved())){
		for(int i = 0; i < trajectory->segmentTimes.size();i++){
			trajectory->segmentTimes[i] +=0.2;
		}
		Eigen::MatrixXd coeffQP =  trajectory->solve(degreeOpt);
		count+=1;
		if(count == 10){
			std::cout << " could not plan flight" << std::endl;
			return false;
		}
	}
	return true;
}

bool ros_replan_utils::initialPlan(int degreeOpt, Eigen::Matrix4d target){
        //std::cout << "initial plan " <<std::endl;
	curr_v =0;
	nav_msgs::Odometry current_heading;
		trajectory->vertices.clear();

	if(odom_l->getCurrOdom(&current_heading)){
		waypoint start(current_heading);
		trajectory->push_back(start);
	}
        //std::cout << "first point pushed " <<std::endl;	
	for (int i =0;i < future_v.size();i++){
		trajectory->push_back(future_v[i]);
	}
        //std::cout << "last point added" <<std::endl;
	trajectory->calcPerchCond(target);
	prevTarget = target;
	fullStop=0;
	//set Time
        //std::cout << " initially generate segment time " <<std::endl; 
	trajectory->autogenTimeSegment();
	segmentTimes = trajectory->segmentTimes;
	int count = 0;
	Eigen::MatrixXd coeffQP =  trajectory->solve(degreeOpt);
	while (!(trajectory->checkSolved())){
		for(int i = 0; i < trajectory->segmentTimes.size();i++){
			trajectory->segmentTimes[i] +=0.2;
		}
		Eigen::MatrixXd coeffQP =  trajectory->solve(degreeOpt);
		count+=1;
		if(count == 10){
			std::cout << " could not plan flight" << std::endl;
			return false;
		}
	}
	return true;
}



//Call this function to replan using the previous target 
bool ros_replan_utils::replan(int degreeOpt, double t_elap, double t_off){
	/*Eigen::Matrix4d Target;
	Target.setIdentity();*/
	int end = future_v.size()-1;
	waypoint last = future_v[end];
	Eigen::VectorXd pos;
	last.getPos(&pos);
	prevTarget(0,3) = pos(0);
	prevTarget(1,3) = pos(1);
	prevTarget(2,3) = pos(2);
	return replan(degreeOpt, t_elap,t_off, prevTarget);
}


//Overload the function in case you have a target detected
bool ros_replan_utils::replan(int degreeOpt, double t_elap, double t_off, Eigen::Matrix4d Target){
	//std::cout << "t elapsed " << t_elap <<std::endl;
        //std::cout << " start replan " <<std::endl;
	prevTarget = Target;
	//Calculate and set the next point
	if(curr_v == future_v.size()){
		//No need to replan the trajectory
		return false;
	}
	//Anticipate your current position 
	//double t_off = 0.0115;
	nav_msgs::Odometry current_heading;
	
	//ros::spinOnce();
	bool use_odom =false; 
	double t0 = double(ros::Time::now().sec) + double(ros::Time::now().nsec)*1e-9 ;
	if(odom_l->getCurrOdom(&current_heading)){
		use_odom = true;
	}
	//std::cout << current_heading.pose.pose << std::endl;
	//while(!odom_l.getCurrOdom(&current_heading)){
	//	ros::spinOnce();
	//}
	//ros::Duration(t_elap*0.25).sleep();
	double t_wait  = double(ros::Time::now().sec) + double(ros::Time::now().nsec)*1e-9 - t0 ;
	waypoint start(current_heading);

	Eigen::MatrixXd point_info = trajectory->evalTraj(t_elap+t_wait);
	Eigen::MatrixXd point_info_2 = trajectory->evalTraj(t_elap+t_wait+t_off);
	Eigen::VectorXd pos(4);
	Eigen::VectorXd odom_pos(4);
	Eigen::VectorXd vel(4);
	Eigen::VectorXd odom_vel(4);
	Eigen::VectorXd accel(4);
	Eigen::VectorXd jerk(4);
	Eigen::VectorXd snap(4) ;
	start.getPos(&odom_pos);
	start.getVelo(&odom_vel);
	for(int i =0; i<4;i++){
		if(use_odom){
			//pos[i] = odom_pos(i);
			//vel[i] = odom_vel(i);
			pos[i] = point_info_2(0,i)-point_info(0,i)+odom_pos(i);
			vel[i] = point_info_2(1,i)-point_info(1,i)+odom_vel(i);
		}
		else{
			pos[i] = point_info_2(0,i);
			vel[i] = point_info_2(1,i);
		}
		accel[i] = point_info_2(2,i);
		jerk[i] = point_info_2(3,i);
		snap[i] = point_info_2(4,i);
	}

	//std::cout << "Pose " << pos.transpose() <<std::endl;
	start.setPos(pos);
	start.setVel(vel);
	start.setAccel(accel);
	start.setJerk(jerk);
	start.setSnap(snap);
	//Save your Previous Trajectory in case we need to revert.
	std::vector<waypoint> vertices_prev =  trajectory->vertices;
	Eigen::MatrixXd coeffSolved_prev = trajectory->coeffSolved;

    std::vector<double> segmentTimes_prev=trajectory->segmentTimes;

	segmentTimes[curr_v]-=(t_elap);

	if((curr_v == future_v.size()-1)&&(segmentTimes[curr_v] < 0.5)){
		curr_v+=1;
		return false;
	}

	if(segmentTimes[curr_v] < 0.5){
		curr_v+=1;
		//consume  the previous segments time if it is less than half a second 
		segmentTimes[curr_v]+=(segmentTimes[curr_v-1])+0.0015;
	}
	//Clear the last trajectories
	trajectory->clearAll();
	trajectory->push_back(start);
	Eigen::Matrix4d targ_heading;
	for (int i = curr_v;i < future_v.size();i++){
		if(i == future_v.size()-1){
			Eigen::Vector4d lastPoint;
			lastPoint(0) = 	Target(0,3) ;
			lastPoint(1) = 	Target(1,3);
			lastPoint(2) =  Target(2,3);
			//std::cout << " Target Used " << lastPoint <<std::endl;
			waypoint last_waypoint(lastPoint);
			future_v[i] = last_waypoint;
		}
		Eigen::VectorXd pos_test;
		future_v[i].getPos(&pos_test);
		trajectory->push_back(future_v[i]);
	}

	trajectory->segmentTimes.clear();

	for (int i = curr_v;i < segmentTimes.size();i++){
		trajectory->segmentTimes.push_back(segmentTimes[i]);
	}
	if(fullStop ==1){
		trajectory->setFullStop();
	}
	else{
		std::cout << " TARGET " << Target <<std::endl;
		trajectory->calcPerchCond(Target);
	}		

	//GENERATE FOV CONDITION
	if(fovEnable){
		int rows = 8;//ceil(((segmentTimes[segmentTimes.size()-1]-0.4)/0.033));
	    //std::cout << " fov start"<<std::endl;
        QP_ineq_const full_ineq_constr;
	    int coeffNum = trajectory->getPolyOrder()*(trajectory->numWaypoints() - 1)*trajectory->getDim() ;
		full_ineq_constr.C =Eigen::MatrixXd::Zero(rows,coeffNum);
		full_ineq_constr.f = Eigen::VectorXd::Constant(rows,-1);
		full_ineq_constr.d = Eigen::VectorXd::Constant(rows,1);
		float t_now = t_elap;
		//std::cout << " rows" << rows << std::endl;
		float fullTime = 0.0;
		for(int i =0;i<segmentTimes.size();i++){
			fullTime+=segmentTimes[i];
		}
		for(int k=0;k<rows;k++){
			QP_ineq_const temp_ineq_constr;
			double incr_time = (fullTime-0.5)/rows;
			t_now +=incr_time;//
			Eigen::MatrixXd replan_pose = trajectory->evalTraj(t_now);
        		Eigen::Vector4d pose_fov;
		        Eigen::Vector3d accelfov;
			for(int i =0; i<4;i++){
				pose_fov[i] = replan_pose(0,i);
				if(i!=3){
					accelfov[i] = replan_pose(2,i);
				}
			}

			Eigen::VectorXd pose_last;
			future_v[future_v.size()-1].getPos(&pose_last);
			Eigen::Vector3d end_point= pose_last.block<3,1>(0,0);	
			//std::cout << " Insert number  " << t_now <<std::endl;
			if (trajectory->genInEqFOV(t_elap,end_point, pose_fov, accelfov, &temp_ineq_constr)){
				//std::cout << "generate FoV"<< std::endl;
				full_ineq_constr.C.block(k, 0,1, coeffNum) = temp_ineq_constr.C.block(0, 0,1, coeffNum);
				///std::cout << "insert C" <<std::endl;
				full_ineq_constr.d(0)= temp_ineq_constr.d(0);
				//std::cout << "insert d" <<std::endl;
				full_ineq_constr.f(0) = temp_ineq_constr.f(0);
				//std::cout << "insert f" <<std::endl;

				//std::cout << " count" << count << std::endl;
				//std::cout << "evaluation: "<<temp_ineq_constr.f(0) <<std::endl;
				//trajectory.setCostVector(temp_ineq_constr.C.block(0, 0,1, coeff).transpose());
			}
		}
		//std::cout << " all values input " <<std::endl;
		trajectory->push_joint_ineq_constr(full_ineq_constr);
	}
	//SOLVE THE MATRIX REVERT IF  NOT SOLVABLE
	int count = 0;
	trajectory->setFullStop();
	Eigen::MatrixXd coeffQP =  trajectory->solve(degreeOpt);
	if(!(trajectory->checkSolved())){
		trajectory->clear_ineq();
		trajectory->clearCostVector();
		coeffQP =  trajectory->solve(degreeOpt);
	}
	//std::cout << "End SM Solve" <<std::endl;
	while (!trajectory->checkSolved()){
		//std::cout << "REPLAN NEED MORE TIME" <<std::endl;
		trajectory->segmentTimes[curr_v] +=0.2;
		Eigen::MatrixXd coeffQP =  trajectory->solve(degreeOpt);
		count+=1;
		if(count == 10){
			//revert to previous trajectory 
			trajectory->overideSolve();
			trajectory->vertices = vertices_prev;
			trajectory->coeffSolved = coeffSolved_prev;
			trajectory->segmentTimes =  segmentTimes_prev;
			std::cout << " could not plan flight" << std::endl;
			return false;
		}
	}
	//std::cout << "successful replanning" <<std::endl;
	return true;
}

void ros_replan_utils::setFOVEnable(bool in){
	fovEnable = in;
}
