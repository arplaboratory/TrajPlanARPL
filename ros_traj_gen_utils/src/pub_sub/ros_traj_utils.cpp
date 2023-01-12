#include <ros_traj_gen_utils/ros_traj_utils.h>

using namespace std;



void ros_traj_utils::graph2D_traj(int derivative_order, TrajBase * traj ){
	double dt = 0.01;
	Eigen::MatrixXd result = traj->calculateTrajectory(derivative_order,  dt);
	std::string title = Deriv_title[derivative_order];
	std::string der_app = append[derivative_order];
	//Record File 
	ofstream outFileX;
	ofstream outFileY;
	ofstream outFileZ;
	ofstream outFileW;
	
	outFileX.open("tempX"+der_app +".dat");
	outFileY.open("tempY"+der_app +".dat");
	outFileZ.open("tempZ"+der_app +".dat");
	outFileW.open("tempW"+der_app +".dat");
	for (int j=0; j< result.rows(); j++){
		double time = j *dt;
		outFileX << time;
		outFileX << " " << result(j,0) << endl;
		outFileY << time;
		outFileY << " " << result(j,1) << endl;
		outFileZ << time;
		outFileZ << " " << result(j,2) << endl;
		outFileW << time;
		outFileW<< " " << result(j,3) << endl;
	}
	outFileX.close();
	outFileY.close();
	outFileZ.close();
	outFileW.close();
	// VISUALIZATION
    GnuplotPipe gp;
	gp.sendLine("set title " + title);
    gp.sendLine("plot 'tempX"+ der_app +".dat' , 'tempY"+ der_app +".dat', 'tempZ"+ der_app +".dat' , 'tempW"+ der_app +".dat' ");
}



void ros_traj_utils::graph2D_traj(int derivative_order, double t_start, TrajBase * traj ){
	double dt = 0.01;
	Eigen::MatrixXd result = traj->calculateTrajectory(derivative_order,  dt);
	std::string title = Deriv_title[derivative_order];
	std::string der_app = append[derivative_order];
	//Record File 
	ofstream outFileX;
	ofstream outFileY;
	ofstream outFileZ;
	ofstream outFileW;
	
	outFileX.open("tempX"+der_app +".dat");
	outFileY.open("tempY"+der_app +".dat");
	outFileZ.open("tempZ"+der_app +".dat");
	outFileW.open("tempW"+der_app +".dat");
	for (int j=0; j< result.rows(); j++){
		double time = j *dt+t_start;
		outFileX << time;
		outFileX << " " << result(j,0) << endl;
		outFileY << time;
		outFileY << " " << result(j,1) << endl;
		outFileZ << time;
		outFileZ << " " << result(j,2) << endl;
		outFileW << time;
		outFileW<< " " << result(j,3) << endl;
	}
	outFileX.close();
	outFileY.close();
	outFileZ.close();
	outFileW.close();
	// VISUALIZATION
    //GnuplotPipe gp;
	//gp.sendLine("set title " + title);
    //gp.sendLine("plot '/home/jeff/traj_logs/tempX"+ der_app +".dat' , '/home/jeff/traj_logs/tempY"+ der_app +".dat', '/home/jeff/traj_logs/tempZ"+ der_app +".dat' , '/home/jeff/traj_logs/tempW"+ der_app +".dat' ");
}

void ros_traj_utils::group_Plot(){
	// VISUALIZATION
    GnuplotPipe gp;
	gp.sendLine("set title 'Pos replanning'");
    gp.sendLine("plot 'tempX0.000000Pos.dat' , 'tempX0.400000Pos.dat', '/tempX0.800000Pos.dat' , 'tempX1.200000Pos.dat'" );
	/*, '/home/jeff/traj_logs/tempY0.000000Pos.dat' , '/home/jeff/traj_logs/tempY0.400000Pos.dat',  '/home/jeff/traj_logs/tempY0.800000Pos.dat' , '/home/jeff/traj_logs/tempY1.200000Pos.dat' ,	 '/home/jeff/traj_logs/tempZ0.000000Pos.dat' , '/home/jeff/traj_logs/tempZ0.400000Pos.dat',  '/home/jeff/traj_logs/tempZ0.800000Pos.dat' , '/home/jeff/traj_logs/tempZ1.200000Pos.dat' ");
*/}




nav_msgs::Path ros_traj_utils::encodePath(int derivOrder, TrajBase * traject, const std::string& frame_id) {
    nav_msgs::Path msg;
	ros::NodeHandle nh;
    double dt = 0.01;
	bool display2D = false ; 
	Eigen::MatrixXd position = traject->calculateTrajectory(derivOrder,dt);
    msg.header.frame_id = frame_id;
    msg.header.stamp = ros::Time::now();
	geometry_msgs::Quaternion rot;
	rot.x = 0;
	rot.y = 0;
	rot.z = 0;
	rot.w = 1;
	/*nh.getParam("/display_2D", display2D);
	if(display2D){
				graph2D_traj(1, traject );
				graph2D_traj(2, traject );
				graph2D_traj(3, traject );
				graph2D_traj(4, traject);
				graph2D_traj(0, traject);
				ros_b3_utils::graph2DB3(traject);
			}*/

    for (int k = 0; k < position.rows(); k++) {
			geometry_msgs::PoseStamped ps;
			geometry_msgs::Pose pose;
			geometry_msgs::Point point;

			point.x = position(k,0);
			point.y = position(k,1);
			point.z = position(k,2);
			if (traject->getDim() ==4){
				Quaternion q = ToQuaternion( position(k,3), 0.0, 0.0); 
				rot.x = q.x;
				rot.y = q.y;
				rot.z = q.z;
				rot.w = q.w;
			}
			pose.position = point;
			pose.orientation = rot;
			ps.pose = pose;
			ps.header.frame_id =  frame_id;
			double time = (double) k *dt;
			ps.header.stamp = ros::Time(time);
			msg.poses.push_back(ps);
    }
	return msg;
}



visualization_msgs::MarkerArray  ros_traj_utils::visualize(TrajBase * traj, const std::string& frame_id){
	visualization_msgs::MarkerArray Markerarr;
	Eigen::MatrixXd pos = traj->calculateTrajectory( 0,  0.025);
	Eigen::MatrixXd acc = traj->calculateTrajectory( 2,  0.025);
	for(int j=1; j<  pos.rows();j++){
		double b3x = acc(j,0) ;
		double b3y = acc(j,1) ;
		double b3z = acc(j,2) +9.8;
		double norm = sqrt(b3x*b3x+b3y*b3y+b3z*b3z);
		b3x = b3x/norm;
		b3y = b3y/norm;
		b3z = b3z/norm;
		visualization_msgs::Marker marker;
		marker.header.frame_id = frame_id;
		marker.header.stamp = ros::Time::now();
		marker.type = 0;
		marker.id = j;
		marker.header.seq = j;
		marker.color.r  = 1.0;
		marker.color.a  = 1.0;
		marker.scale.x = 0.025;
		marker.scale.y = 0.075;
		marker.scale.z = 0.05;
		geometry_msgs::Point start;
		start.x =  pos(j,0) ;
		start.y =  pos(j,1) ;
		start.z =  pos(j,2) ;
		geometry_msgs::Point end;
		end.x =  start.x + 0.3*b3x ;
		end.y =  start.y + 0.3*b3y ;
		end.z =  start.z + 0.3*b3z ;
		marker.points.push_back(start);
		marker.points.push_back(end);
		Markerarr.markers.push_back(marker);
	}
	return Markerarr;
}

void ros_traj_utils::graph2DB3( TrajBase  * traj){
	double dt = 0.001;
	Eigen::MatrixXd result = traj->calculateTrajectory( 2,  dt);
	//Record File 
	ofstream outFileX;
	ofstream outFileY;
	ofstream outFileZ;
	outFileX.open("B3X.dat");
	outFileY.open("B3Y.dat");
	outFileZ.open("B3Z.dat");
	for (int j=1; j<  result.rows(); j++){
		double time = j *dt;
		double x = result(j,0) ;
		double y = result(j,1) ;
		double z = result(j,2) +9.8;
		double norm = sqrt(x*x+y*y+z*z);
		x = x/norm;
		y = y/norm;
		z = z/norm;
		outFileX << time;
		outFileX << " " << x << endl;
		outFileY << time;
		outFileY << " " << y<< endl;
		outFileZ << time;
		outFileZ << " " << z << endl;
	}
	outFileX.close();
	outFileY.close();
	outFileZ.close();

	// VISUALIZATION
    GnuplotPipe gp;
	gp.sendLine("set title 'B3 axis' " );
    gp.sendLine("plot 'B3X.dat' , 'B3Y.dat', 'B3Z.dat' ");
}


