#include <traj_gen/trajectory/TrajBase.h>
#include <math.h>       
using namespace std;

TrajBase::TrajBase()
{
}



bool TrajBase::condCheck() {
	if(vertices.size() >1){
		return true;
	}
	return false;
}
TrajBase::~TrajBase()
{
}

void TrajBase::setVertices(std::vector<waypoint> w){
	vertices = w;
}

void TrajBase::setTimeSegment(std::vector<double> t)
{
	segmentTimes = t;
}

void TrajBase::equalTimeSegment(double duration)
{
	int count = 0;
	int length = vertices.size()-1;
	double dt = duration/length;
	std::vector<double> times(length);
	while (count < length) {
		count = count + 1;
		times[count-1] = dt;
	}
	segmentTimes = times;
}

float TrajBase::autogenTimeSegment()
{	
	float totalTime = 0.0;
	double v_max = limits[1];
	double a_max = limits[2];
	double magic_fabian_constant = 6.5;
	double yaw_dist = 0.0;
	const double yaw_rate = 2.0;
	int length = vertices.size()-1;
	std::vector<double> times(length);
	//double currTime = 0.0;
	for (int i = 0; i < length; i++) {
		Eigen::VectorXd currPoint;
		Eigen::VectorXd prevPoint;
		vertices[i].getPos(&prevPoint);
		vertices[i+1].getPos(&currPoint);
		//Give enough time for the yaw turns
		double yaw_time = abs(currPoint[3] -prevPoint[3])*2; 
		//Yaw angle does not matter for distance are optimal time
		currPoint[3] =0;
		prevPoint[3]=0;

		double distance = (currPoint - prevPoint).norm();
		double t = 1.1* distance / v_max * 2 *
				   (1.0 + magic_fabian_constant * v_max / a_max *
							  exp(-distance / v_max * 2));
   if(t<yaw_time){
			t = yaw_time;
		}
		times[i] = t;
		std::cout <<  " ALlocated time " << times[i] <<std::endl;
		totalTime+=t;
	  }
	segmentTimes = times;
	return totalTime;
}


void TrajBase::push_back(waypoint w){
	if (w.getDim() != dim){
		//Thrown an exception. Waypoints must match the trajBase dimension
		std::cout << "An exception occurred. Traj dimension and waypoint must have the same dimension " << '\n';
		throw 20;
	}
	/*if(vertices.size()>=1){ yaw reversal code not to be considered for now 
		waypoint waypoint_prev =vertices[vertices.size()-1];
		Eigen::VectorXd last_pos;
		Eigen::VectorXd curr_pos;
		waypoint_prev.getPos(&last_pos);
		w.getPos(&curr_pos);
		while((curr_pos[3] - last_pos[3]) > 3.1415926){
			curr_pos[3] -= 2*3.1415926;
		}
		while((curr_pos[3] - last_pos[3]) < -3.1415926) {
			curr_pos[3]   += 2*3.1415926;
		} 
		w.setPos(curr_pos);      
	}*/
	vertices.push_back(w);
}

int TrajBase::getDim() {
	return dim;
}

waypoint TrajBase::getWaypoint(int t){
	if ( t >= vertices.size()){
		//Thrown an exception. Waypoints must match the trajBase dimension
		std::cout << "An exception occurred. Accessinig index of waypoints bigger than list  "<< '\n';
		std::cout << "num waypoints   "<< vertices.size()<<"Index access " <<t<< '\n';
		throw 20;
	}
	return vertices[t];
}
void TrajBase::setPolyOrder(int p){
	polyOrder = p;
}

int TrajBase::getPolyOrder() {
	return polyOrder;
}

int TrajBase::numWaypoints() {
	return vertices.size();
}

void TrajBase::setFullStop(){
	int numPoint = vertices.size();
	vertices[numPoint-1].setFullStop();
}


void TrajBase::push_ineq_constr(QP_ineq_const constr,int d){
	if(add_eq_constr.size()==0){
		for(int i=0;i<dim;i++){
			QP_ineq_const temp_ineq_constr;
			add_ineq_constr.push_back(temp_ineq_constr);
		}
	}
	QP_ineq_const temp_ineq_constr = add_ineq_constr[d];
	if(temp_ineq_constr.d.rows()!=0){
		//Verticall stack the two objects
		int rows = temp_ineq_constr.d.rows()+constr.d.rows();
		int cols = constr.C.cols();
		Eigen::VectorXd d = Eigen::VectorXd::Zero(rows);
		Eigen::VectorXd f = Eigen::VectorXd::Zero(rows);
		Eigen::MatrixXd C = Eigen::MatrixXd::Zero(rows,cols);
		//vertical stacking equality constraints 
		d.head(temp_ineq_constr.d.rows()) = temp_ineq_constr.d;
		d.tail(constr.d.rows()) = constr.d;
		f.head(temp_ineq_constr.f.rows()) = temp_ineq_constr.f;
		f.tail(constr.f.rows()) = constr.f;
		C.block(0,0,temp_ineq_constr.d.rows(),cols) = temp_ineq_constr.C;
		C.block(0,temp_ineq_constr.d.rows(),constr.d.rows(),cols) = constr.C;
		temp_ineq_constr.d = d;
		temp_ineq_constr.f = f;
		temp_ineq_constr.C = C;
	}
	else{
		temp_ineq_constr = constr;
	}
	add_ineq_constr[d] = temp_ineq_constr;
}
void TrajBase::push_eq_constr(QP_constraint constr, int d){
	if(add_eq_constr.size()==0){
		for(int i=0;i<dim;i++){
			QP_constraint temp_eq_constr;
			add_eq_constr.push_back(temp_eq_constr);
		}
	}
	QP_constraint temp_eq_constr = add_eq_constr[d];
	if(temp_eq_constr.b.rows()!=0){
		//Verticall stack the two objects
		int rows = temp_eq_constr.b.rows()+constr.b.rows();
		int cols = constr.a.cols();
		Eigen::VectorXd b = Eigen::VectorXd::Zero(rows);
		Eigen::MatrixXd A = Eigen::MatrixXd::Zero(rows,cols);
		//vertical stacking equality constraints 
		b.head(temp_eq_constr.b.rows()) = temp_eq_constr.b;
		b.tail(constr.b.rows()) = constr.b;
		A.block(0,0,temp_eq_constr.b.rows(),cols) = temp_eq_constr.a;
		A.block(0,temp_eq_constr.b.rows(),constr.b.rows(),cols) = constr.a;
		temp_eq_constr.b = b;
		temp_eq_constr.a = A;
	}
	else{
		temp_eq_constr = constr;
	}
	add_eq_constr[d] = temp_eq_constr;

}

void TrajBase::push_joint_ineq_constr(QP_ineq_const constr){
	add_joint_ineq_constr = constr;
}

void TrajBase::push_joint_eq_constr(QP_constraint constr){
	if(add_joint_eq_constr.b.rows()==0){
		add_joint_eq_constr = constr;
	}
	else{
		int rows = add_joint_eq_constr.b.rows()+constr.b.rows();
		int cols = constr.a.cols();
		Eigen::VectorXd b = Eigen::VectorXd::Zero(rows);
		Eigen::MatrixXd A = Eigen::MatrixXd::Zero(rows,cols);
		//vertical stacking equality constraints 
		b.head(add_joint_eq_constr.b.rows()) = add_joint_eq_constr.b;
		b.tail(constr.b.rows()) = constr.b;
		A.block(0,0,add_joint_eq_constr.b.rows(),cols) = add_joint_eq_constr.a;
		A.block(0,add_joint_eq_constr.b.rows(),constr.b.rows(),cols) = constr.a;
		add_joint_eq_constr.b = b;
		add_joint_eq_constr.a = A;
	
	}
}

void TrajBase::clear_ineq(){
	add_ineq_constr.clear();
	QP_ineq_const temp_ineq_constr;
	add_joint_ineq_constr = temp_ineq_constr;
}

void TrajBase::clear_eq(){
	add_eq_constr.clear();
	QP_constraint temp_eq_constr;
	add_joint_eq_constr = temp_eq_constr;
}

void TrajBase::clear_vertices(){
	for (int i = 0; i < traj_valid.size();i++){
		traj_valid[i] = false;
	}

	vertices.clear();
}

void TrajBase::clearAll(){
	segmentTimes.clear();
	clearCostVector();
	clear_eq();
	clear_ineq();
	clear_vertices();
}

void TrajBase::clearCostVector(){
	useCostVector=false;
}

bool TrajBase::checkSolved(){
	if(traj_valid.size()==0){
		return false;
	}
	for(int i =0;i<traj_valid.size();i++){
		if (!traj_valid[i]){
			return false;
		}
	}
	return true;
}

void TrajBase::overideSolve(){
	for(int i =0;i<traj_valid.size();i++){
		if (traj_valid[i]!=1){
			traj_valid[i] = true;
		}
	}
}

//Given a Homogenous Transform and a forward Velocity how to convert to this system
void TrajBase::calcPerchCond(Eigen::Matrix4d H){
    double check = impV.norm();
	int numPoint = vertices.size();
	//Autoset the vector if it is not assigned first 
	if((check) < 1e-3){
		//Vs1,0,Vs3,0
		impV << 1.0, 0.0 , -3.0, 0.0;
	}
	vertices[numPoint-1].setVel(impV);
	//tangent opposite over adjuacnet 
	//std::cout << H <<std::endl;
	double norm = sqrt(H(0,2)*H(0,2)+H(1,2)*H(1,2));
	double pitch = atan2(norm,H(2,2));
	std::cout << "Pitch " << pitch <<std::endl;
	if(abs(pitch) < 0.5 ){
		Eigen::VectorXd zeroV(4);
		zeroV << 0,0,0,0;
		vertices[numPoint-1].setVel(zeroV);
	}

	std::cout << " number of vertice " << numPoint <<std::endl;
    double force = 4.0*sin(pitch);
	//std::cout << "Number of points" <<numPoint <<std::endl;
	Eigen::VectorXd pos1;
	Eigen::VectorXd pos2; 
	Eigen::VectorXd finalAccel = Eigen::VectorXd::Zero(4);
	waypoint lastWaypoint = vertices[numPoint-1];
	waypoint seclastWay = vertices[numPoint - 2];
	std::cout << "initial calc " <<std::endl;
    lastWaypoint.getPos(&pos1);
	seclastWay.getPos(&pos2);
	//Set the acceleration based on the 3rd vector of the target
	finalAccel[0] = H(0,2)*force;
	finalAccel[1] = H(1,2)*force;
	//DO NOT LET THE PERCHING TILT PAST 90 Degrees
	if(H(2,2) <0){
		H(2,2) = 0;
	}
	finalAccel[2] = H(2,2)*force -9.81;
    std::cout << "set acceleration " <<std::endl;
	vertices[numPoint-1].setAccel(finalAccel);
	//vertices[numPoint-1].setJerk(Eigen::VectorXd::Zero(4));
	//vertices[numPoint-1].setSnap(Eigen::VectorXd::Zero(4));
		
	waypoint_ineq_const ineq_const_a;
	Eigen::Vector4d numIneqCon = Eigen::VectorXd::Zero(4);
    std::cout << "done acceleartion " <<std::endl;
	Eigen::Vector4d lower_a = Eigen::VectorXd::Zero(4);
	Eigen::Vector4d upper_a = Eigen::VectorXd::Zero(4);
	lower_a[0] = finalAccel[0];
	upper_a[0] = finalAccel[0] + 4;
	lower_a[1] = finalAccel[1] -0.2;
	upper_a[1] = finalAccel[1] +0.2;
	numIneqCon[1] = 1;
	numIneqCon[2] = 1;
	lower_a[2] = finalAccel[2] ;
	upper_a[2] = finalAccel[2] + 3;
	
	//std::cout << "Set Accel " << finalAccel <<std::endl;
		
	ineq_const_a.derivOrder = 2;
	ineq_const_a.lower = lower_a;
	ineq_const_a.upper = upper_a;
	ineq_const_a.InEqDim = numIneqCon;
	/*
	std::cout << "Inequality on and off " << numIneqCon <<std::endl;
	
	
	std::cout << "Lower V " << lower_v <<std::endl;
	std::cout << "upper V " << upper_v <<std::endl;
	
	std::cout << "Lower A " << lower_a <<std::endl;
	std::cout << "upper A " << upper_a <<std::endl;
	*/
    std::cout << "PUSH CONSTRAINTS" <<std::endl;
	vertices[numPoint-1].ineq_constraint.push_back(ineq_const_a);
	//if(constrainV){
		//vertices[numPoint-1].ineq_constraint.push_back(ineq_const_v);
	//}
      //  std::cout << " push constraint done" <<std::endl;
	//set no velocity if our pitch angle is low 
}

//Simple calculation for X aligned perching 
void TrajBase::calcPerchCond(double pitch){
	Eigen::Matrix4d H;
	H(0,2) = -1*sin(pitch);
	H(1,2) = 0;
	H(2,2) = cos(pitch);
	calcPerchCond(H);
}

/*Virtual Stubs*/



bool TrajBase::genInEqFOV(double replan_time, Eigen::Vector3d target, 	Eigen::Vector4d pose,	Eigen::Vector3d accel , QP_ineq_const * constraint)
{
        //std::cout << " strat ineq Fov" <<std::endl;
        int coeffNum = (vertices.size() - 1) *  polyOrder;
	QP_ineq_const ineq_const;
	ineq_const.d= Eigen::VectorXd::Zero(4);
	ineq_const.f= Eigen::VectorXd::Zero(4);
	ineq_const.C = Eigen::MatrixXd::Zero(4, coeffNum*dim);
	Eigen::MatrixXd const_conv =  Eigen::MatrixXd::Zero(6, coeffNum*dim);
	//std::cout << " fov start " <<std::endl;
        FOV_constraint fov(pose, accel);
        //std::cout << " start derovative fov " <<std::endl;
	fov_constr constr = fov.derivative_FOV(target);
	//SGD Bound check to see if it is 
	//Eigen::VectorXd sgd_step = 0.1*constr.Jacobian.normalized();
	//for(int i =0;i<3;i++){
	//	pose[i] -=sgd_step[i];
	//	accel[i] -=sgd_step[i+3];
	//}
        //std::cout << "fov bound " <<std::endl;
	//FOV_constraint fov_bound(pose, accel);
	//fov_zero_order checker = fov_bound.fov_eval(target);
	//std::cout << " fov check strat" <<std::endl;
	//if(-10 >= checker.right-checker.left){
	//	return false;
	//}
	//Else we do the calcualtion to add the full inequality constriant
	double sum_jacobian = 1e-8; //Small Nudge to avoid singularities 
	double sum_jacob_frac = 0.0;
	ineq_const.f(0) = constr.diff;//*sum_jacob_frac/sum_jacobian;
	ineq_const.d(0) = -50000;
	Eigen::MatrixXd row_pos = basis(replan_time, 0).transpose();
	Eigen::MatrixXd row_acc = basis(replan_time, 2).transpose();
	Eigen::MatrixXd row_jerk = basis(replan_time, 3).transpose();
	for(int i = 0; i < 3; i++){
		ineq_const.f(i+1) = 15;//*sum_jacob_frac/sum_jacobian;
		ineq_const.d(i+1) = -15;
		ineq_const.C.block(i+1, coeffNum*i,1, polyOrder) = row_jerk;
		//std::cout << "loop: " << i << std::endl;
		//std::cout << ineq_const.C <<std::endl;
		const_conv.block(i, coeffNum*i,1, polyOrder) = row_pos; 
		const_conv.block(i+3, coeffNum*i,1, polyOrder) = row_acc; 
		//Goal [posx ; posy; posz ; accx; accy; accz]
	}
	
	//std::cout <<"constraint Jacobian " << constr.Jacobian.rows() <<std::endl;
	ineq_const.C.block(0, 0,1, coeffNum*dim) = constr.Jacobian *const_conv;
	*constraint = ineq_const ;
	return true;
}

