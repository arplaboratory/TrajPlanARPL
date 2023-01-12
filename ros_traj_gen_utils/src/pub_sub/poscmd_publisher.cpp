#include <ros_traj_gen_utils/poscmd_publisher.h>
using namespace std;



poscmd_publisher::poscmd_publisher(std::string cmd_topic, ros::Timer * timer, double dt ){
	ros::NodeHandle nh;
	pubCMD = nh.advertise<quadrotor_msgs::PositionCommand>(cmd_topic,10);
	state =END;
	*timer = nh.createTimer(ros::Duration(dt), &poscmd_publisher::timerCallback , this);
}



void poscmd_publisher::timerCallback(const ros::TimerEvent& event){
	if(state==END){
		return;// no flight
	}
	quadrotor_msgs::PositionCommand point;
	if(state==FLIGHT){
		ros::Time now = ros::Time::now();	
		double traj_time = (now-begin).toSec();		
		if(traj_time >= (totalTime)){
			traj_time = totalTime;
			state = HOVER;
		}
		//std::cout <<traj_time <<std::endl;

		Eigen::MatrixXd pt;
		pt = currTraj->evalTraj(traj_time);
		geometry_msgs::Point posXYZ;
		posXYZ.x = pt(0,0);
		posXYZ.y = pt(0,1);
		posXYZ.z = pt(0,2);
		geometry_msgs::Vector3 veloXYZ;
		veloXYZ.x = pt(1,0);
		veloXYZ.y = pt(1,1);
		veloXYZ.z = pt(1,2);
		geometry_msgs::Vector3 accelXYZ;
		accelXYZ.x = pt(2,0);
		accelXYZ.y = pt(2,1);
		accelXYZ.z = pt(2,2);
		geometry_msgs::Vector3 jerkXYZ;
		jerkXYZ.x = pt(3,0);
		jerkXYZ.y = pt(3,1);
		jerkXYZ.z = pt(3,2);
		point.position = posXYZ;
		point.velocity = veloXYZ; 
		point.acceleration = accelXYZ;
		point.jerk =  jerkXYZ;
		point.yaw = 0;//pt(0,3);
		point.yaw_dot = 0; // pt(1,3);
		point.kx[0] = kx;
		point.kv[0] = kv;
		point.kx[1] = kx;
		point.kv[1] = kv;
		point.kx[2] = kx;
		point.kv[2] = kv;
		point.header.frame_id = frame_id;
		//std::cout << count <<std::endl;
		//if(traj_time >= (currTraj.segmentTimes.back())){
		finalState = point;
		if(totalTime- traj_time < 0.01){
			ros::NodeHandle nh;
			bool motorsOff;
			//nh.getParam("/trajectory_gen_demo/motorOff", motorsOff);
			//if(motorsOff){
			nh.setParam("/quadrotor/quadrotor_simulator_so3/enableUnity",false);
			//}
		}
			//Set the final state to the last point 
		//}
		//else{
			//point = flightTraj[count];
			//position_cmd_history.push_back(point);
			//count+=1;
			//std::cout << count <<std::endl;
		//}
		
	}
	if(state==HOVER){
		point = finalState;
	}
    point.header.stamp = ros::Time::now();
	pubCMD.publish(point);
}


void poscmd_publisher::setNewFlightPath( TrajBase * traj){
		count = 0; //reset count
		currTraj = traj;
		totalTime = 0.0;
		for(int i=0;i<traj->segmentTimes.size();i++){
			totalTime+=traj->segmentTimes[i];
		}
		//Round down the total time 
		totalTime = totalTime/0.01;
		totalTime = floor(totalTime);
		totalTime = totalTime*0.01;
		begin = ros::Time::now();
		state = FLIGHT;
}



int poscmd_publisher::getState(){
	return state;
}

void poscmd_publisher::setEND(){
	state =END;
}


//Static Function
void poscmd_publisher::startFlight(TrajBase * traj){
	setNewFlightPath(traj);
}

std::vector<quadrotor_msgs::PositionCommand> poscmd_publisher::arplCMDlist(double dt, double kx, double kv, std::string frame_id, TrajBase * traj){
	std::vector<quadrotor_msgs::PositionCommand>  posCMD;
	Eigen::MatrixXd pos =  traj->calculateTrajectory( 0, dt);
	Eigen::MatrixXd velo =  traj->calculateTrajectory( 1, dt);
	Eigen::MatrixXd accel =  traj->calculateTrajectory( 2, dt);
	Eigen::MatrixXd jerk =  traj->calculateTrajectory( 3, dt);
	//double totalTime = traj.segmentTimes[traj.segmentTimes.size()-1];
	double totalTime = 0.0;
	for(int i =0;i<traj->segmentTimes.size();i++){
		totalTime+=traj->segmentTimes[i];
	}
	for(int j =0; j < totalTime/dt; j++){
		quadrotor_msgs::PositionCommand point;
		geometry_msgs::Point posXYZ;
		posXYZ.x = pos(j,0);
		posXYZ.y = pos(j,1);
		posXYZ.z = pos(j,2);
		geometry_msgs::Vector3 veloXYZ;
		veloXYZ.x = velo(j,0);
		veloXYZ.y = velo(j,1);
		veloXYZ.z = velo(j,2);
		geometry_msgs::Vector3 accelXYZ;
		accelXYZ.x = accel(j,0);
		accelXYZ.y = accel(j,1);
		accelXYZ.z = accel(j,2);
		geometry_msgs::Vector3 jerkXYZ;
		jerkXYZ.x = jerk(j,0);
		jerkXYZ.y = jerk(j,1);
		jerkXYZ.z = jerk(j,2);
		point.position = posXYZ;
		point.velocity = veloXYZ; 
		point.acceleration = accelXYZ;
		point.jerk =  jerkXYZ;
		point.yaw = pos(j,3);
		point.yaw_dot = velo(j,3);
		point.kx[0] = kx;
		point.kv[0] = kv;
		point.kx[1] = kx;
		point.kv[1] = kv;
		point.kx[2] = kx;
		point.kv[2] = kv;
		point.header.frame_id = frame_id;
		posCMD.push_back(point);
	}
	return posCMD;
}


