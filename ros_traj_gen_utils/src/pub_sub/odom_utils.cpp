#include <ros_traj_gen_utils/odom_utils.h>
using namespace std;

void odom_utils::outputListiner(const nav_msgs::Odometry &msg){
	//std::cout << ros::Time::now() <<std::endl;
	if(enable_write){
		//Lock the read to be at now and not before
		if(enable_time_samp){
			double read_time = msg.header.stamp.toSec();
			if(now > read_time){
				std::cout << "Dropping values" <<std::endl;
				return; 
			}				
		}
		current_heading = msg;
		read = true; 
	}
}

bool odom_utils::getCurrOdom(nav_msgs::Odometry * curr_heading){
	if(read){
		*curr_heading = current_heading;
		//read = false;
		return true;
	}
	return read;
}