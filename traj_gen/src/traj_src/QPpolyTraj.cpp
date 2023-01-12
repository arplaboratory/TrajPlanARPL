#include <traj_gen/trajectory/QPpolyTraj.h>
using namespace std;
#include <ctime>
#include <ros/ros.h>
#include <Eigen/Dense>

using namespace Eigen;

QPpolyTraj::QPpolyTraj()
{    	
	dim =4;
	limits = Eigen::VectorXd::Zero(5);
	limits(1)=2.0;
	limits(2)=2.0;
}


QPpolyTraj::QPpolyTraj(int d)
{    	dim = d;
		limits = Eigen::VectorXd::Zero(5);
			limits(1)=2.0;
	limits(2)=2.0;
}

QPpolyTraj::QPpolyTraj(int d,std::vector<double> time)
{
	dim = d;
    segmentTimes = time;
	limits = Eigen::VectorXd::Zero(5);
		limits(1)=2.0;
	limits(2)=2.0;
}




//Fast Solve is a quicker solver at the cost of being not the perfect outcome
Eigen::MatrixXd QPpolyTraj::solve(int minDeriv)
{
	// Declare variables and zero memories
	 if(!condCheck()){
		std::cout << "NOT ENOUGH VERTICES TO GENERATE "<<std::endl;
		std::cout << "number points  " <<vertices.size()<<std::endl;
		Eigen::MatrixXd coeff = Eigen::MatrixXd::Zero(1, 4);
		return coeff;
	 }
	 if(add_joint_ineq_constr.d.rows()+add_joint_eq_constr.a.rows()>0){
		 return SMsolve(minDeriv);
	 }
	if(fast){
		return fastMTSolve( minDeriv);
	}
	else{
		return MTsolve( minDeriv);
	}
}


void  thread_FASTQP(int dimension, Eigen::MatrixXd D, int vectorSize, QP_constraint qp,
   Eigen::MatrixXd C, Eigen::VectorXd d,Eigen::VectorXd g0, Eigen::MatrixXd* coeff){
    Eigen::VectorXd sol = Eigen::VectorXd::Zero(vectorSize);
	//Create Copy since the quadprog changes the X&T Q  X matrix each run
	Eigen::MatrixXd A = qp.a, b = qp.b;
	Eigen::MatrixXd Obj = D;
	QP::solve_quadprog(Obj, g0, 
			A.transpose(),-1*b,  
			C.transpose(), d, 
			sol);
	for (int i =0;i<vectorSize;i++){
		coeff->operator()(i,dimension)  = sol[i];
	}
	
}

//EIGEN QP
Eigen::MatrixXd QPpolyTraj::fastMTSolve(int minDeriv)
{
    //Cut the object into 3 vectors
    //We need 3 things for our QP Programming
     // One Q for X^T*Q*X to minimize
     // Two b = A*x constraint to follow
     //For now we consider the constraints
	int coeffNum = (vertices.size() - 1) *  polyOrder;
	 if(!condCheck()){
		std::cout << "NOT ENOUGH VERTICES TO GENERATE "<<std::endl;
		std::cout << "number points  " <<vertices.size()<<std::endl;
		Eigen::MatrixXd coeff = Eigen::MatrixXd::Zero(coeffNum, dim);
		return coeff;
	 }
	 std::vector<boost::thread> threads;
    int numberSegments = segmentTimes.size();
	// Declare variables and zero memories
	//calculating the number of constraints
	int numConstraint = 6 * vertices.size() - 2;
	for (int i = 1; i <vertices.size()-1; i++){
		int addConstraint = vertices[i].getStatus().sum() -1;
		numConstraint +=addConstraint;
	}		
	Eigen::MatrixXd coeff = Eigen::MatrixXd::Zero(coeffNum, dim);
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero((numConstraint), coeffNum);
    Eigen::VectorXd d = Eigen::VectorXd::Zero(numConstraint);
   //generate object function
   	Eigen::MatrixXd D = generateObjFun(minDeriv)+0.001*Eigen::MatrixXd::Identity(coeffNum, coeffNum);
	Eigen::VectorXd g0 = Eigen::VectorXd::Zero(coeffNum);
	ooqpei::OoqpEigenInterface solver();
	for (int j = 0; j < dim; j++){
		QP_constraint qp = genConstraint( j,numConstraint); //each dimension has its unique equality constraint 
		threads.push_back(boost::thread(thread_FASTQP, j,  D, coeffNum, qp,C,d,g0,&coeff));
	}
  for (auto &th : threads) {
    th.join();
  }
 	coeffSolved = coeff;
	//quadprog has no failure as matrix inversion
	  traj_valid[0] = true; 
		traj_valid[1] = true; 
		traj_valid[2] = true; 
		traj_valid[3] = true; 
    return coeff;
}


//SIngle matrix solve
Eigen::MatrixXd QPpolyTraj::SMsolve(int minDeriv)
{
    //Cut the object into 3 vectors
	// Declare variables and zero memories
    int coeffNum = (vertices.size() - 1) *  polyOrder;
		Eigen::MatrixXd coeff(coeffNum, dim);
	 if(!condCheck()){
		std::cout << "NOT ENOUGH VERTICES TO GENERATE "<<std::endl;
		std::cout << "number points  " <<vertices.size()<<std::endl;
		Eigen::MatrixXd coeff = Eigen::MatrixXd::Zero(coeffNum, dim);
		return coeff;
	 }
	//std::cout << " Generate joint matrix solve " <<std::endl;
	QP_constraint qp_constr =  genJointConstraint();
	//std::cout << " generate inequality joint solve" <<std::endl;
	QP_ineq_const qp_ineq_constr =  genInEqJointConstraint();
	//std::cout << " generate joint objective solve" <<std::endl;
	Eigen::MatrixXd comp = generateJointObjFun(minDeriv);
	//std::cout << " all matrices generated" <<std::endl;
	/*
	std::cout << "A: "<<qp_constr.a <<std::endl;
	std::cout << "b: "<<qp_constr.b <<std::endl;
	std::cout << "C: "<<qp_ineq_constr.C <<std::endl;
	std::cout << "d: "<<qp_ineq_constr.d <<std::endl;
	std::cout << "f: "<<qp_ineq_constr.f <<std::endl;*/
	//There is definitely a problem here Trajectory generation fails a second time for some reason....
	Eigen::SparseMatrix<double, Eigen::RowMajor> Obj = comp.sparseView(); 
	Eigen::SparseMatrix<double, Eigen::RowMajor> AooQP = qp_constr.a.sparseView();
	Eigen::SparseMatrix<double, Eigen::RowMajor> C = qp_ineq_constr.C.sparseView();
    	Eigen::VectorXd sol = Eigen::VectorXd::Zero(coeffNum*dim);
	Eigen::VectorXd g0;
	if(useCostVector){
		g0 = costVector;
	}
	else{
		g0 = Eigen::VectorXd::Zero(coeffNum*dim);
	}
	//Eigen::VectorXd g0  = qp_ineq_constr.C.transpose();
	const bool ignoreUnknownError = false;
  /*!
   * Solve min 1/2 x' Q x + c' x, such that A x = b, and d <= Cx <= f
   * @param [in] Q a symmetric positive semidefinite matrix (nxn)
   * @param [in] c a vector (nx1)
   * @param [in] A a (possibly null) matrices (m_axn)
   * @param [in] b a vector (m_ax1)
   * @param [in] C a (possibly null) matrices (m_cxn)
   * @param [in] d a vector (m_cx1)
   * @param [in] f a vector (m_cx1)
   * @param [out] x a vector of variables (nx1)
   * @return true if successful
   */
    //std::cout << " qp_constr.b" <<std::endl;
    // std::cout <<  qp_constr.b <<std::endl;
    if(ooqpei::OoqpEigenInterface::solve(Obj, g0 ,
                   AooQP, qp_constr.b,
                    C,
                    qp_ineq_constr.d,qp_ineq_constr.f,
					sol,ignoreUnknownError)){
		traj_valid[0] = true; 
		traj_valid[1] = true; 
		traj_valid[2] = true; 
		traj_valid[3] = true; 
	}
	else{
		//std::cout << "QP Failed generation" << std::endl;
		traj_valid[0] = false; 
		traj_valid[1] = false; 
		traj_valid[2] = false; 
		traj_valid[3] = false; 		
	}
    /*
	ooqpei::OoqpEigenInterface::solve(Obj, g0, 
			AooQP,btotal,  
			l,u, 
			);*/
	for (int j = 0; j < dim; j++){
		for (int i =0;i<coeffNum;i++){
			coeff(i,j) = sol[i+j*coeffNum];
		}
	}
	coeffSolved = coeff;
    return coeff;
}



void  thread_QP(int dimension, Eigen::MatrixXd Qobj, int coeffNum, QP_constraint qp,
   QP_ineq_const ineq_qp, Eigen::MatrixXd* coeff, std::vector<bool>* traj_valid){
	const bool ignoreUnknownError = false;
    Eigen::VectorXd sol = Eigen::VectorXd::Zero(coeffNum);
	Eigen::VectorXd g0 = Eigen::VectorXd::Zero(coeffNum);
	//Create Copy since the quadprog changes the X&T Q  X matrix each run
	Eigen::SparseMatrix<double, Eigen::RowMajor> Obj = Qobj.sparseView();
	Eigen::MatrixXd A = qp.a, b = qp.b;
	Eigen::SparseMatrix<double, Eigen::RowMajor> AooQP = A.sparseView();
	Eigen::SparseMatrix<double, Eigen::RowMajor> C = ineq_qp.C.sparseView();
  /*!
   * Solve min 1/2 x' Q x + c' x, such that A x = b, and d <= Cx <= f
   * @param [in] Q a symmetric positive semidefinite matrix (nxn)
   * @param [in] c a vector (nx1)
   * @param [in] A a (possibly null) matrices (m_axn)
   * @param [in] b a vector (m_ax1)
   * @param [in] C a (possibly null) matrices (m_cxn)
   * @param [in] d a vector (m_cx1)
   * @param [in] f a vector (m_cx1)
   * @param [out] x a vector of variables (nx1)
   * @return true if successful
   */
   /*
    std::cout << "Obj: " << Obj << std::endl;
	std::cout << "g0: " << g0 << std::endl;
   	std::cout << "AOOQP: " << AooQP << std::endl;
	std::cout << "B: " << b << std::endl;
   	std::cout << "C: " << C << std::endl;
	std::cout << "d: " << ineq_qp.d << std::endl;
   	std::cout << "f: " << ineq_qp.f << std::endl;
	*/
    if(ooqpei::OoqpEigenInterface::solve(Obj, g0 ,
                   AooQP, b,
                    C,
                    ineq_qp.d,ineq_qp.f,
					sol,ignoreUnknownError)){
		//std::cout << "QP successful generation" << std::endl;
		traj_valid->operator[](dimension) = true; 
	}
	else{
		std::cout << "QP Failed generation" << std::endl;
		
	}
    //Eigen::MatrixXd Cost = sol.transpose() * Obj * sol;
	//std::cout << "QP cost:" << Cost<< std::endl;
	for (int i =0;i<coeffNum;i++){
		coeff->operator()(i,dimension)  = sol[i];
	}
}

Eigen::MatrixXd QPpolyTraj::MTsolve(int minDeriv)
{
    //Cut the object into 3 vectors
    //We need 3 things for our QP Programming
     // One Q for X^T*Q*X to minimize
     // Two b = A*x constraint to follow
     //For now we consider the constraints
    int numberSegments = segmentTimes.size();
    int coeffNum = (vertices.size() - 1) *  polyOrder;
	 if(!condCheck()){
		std::cout << "NOT ENOUGH VERTICES TO GENERATE "<<std::endl;
		std::cout << "number points  " <<vertices.size()<<std::endl;
		Eigen::MatrixXd coeff = Eigen::MatrixXd::Zero(coeffNum, dim);
		return coeff;
	 }
	 std::vector<boost::thread> threads;
	// Declare variables and zero memories
	int numConstraint = 6 * vertices.size() - 2;
	for (int i = 1; i <vertices.size()-1; i++){
		int addConstraint = vertices[i].getStatus().sum() -1;
		numConstraint +=addConstraint;
	}		
	Eigen::MatrixXd coeff = Eigen::MatrixXd::Zero(coeffNum, dim);
   //generate object function
   	Eigen::MatrixXd D = generateObjFun(minDeriv);
	for (int j = 0; j < dim; j++){
		QP_constraint qp = genConstraint( j,numConstraint); //each dimension has its unique equality constraint 
		QP_ineq_const ineq_qp = genInEqConstraint(j);
		
		threads.push_back(boost::thread(thread_QP, j,  D, coeffNum, qp,ineq_qp,&coeff,&traj_valid));
			//thread_QP( j,  D, coeffNum, qp,ineq_qp,&coeff,&traj_valid);
	} 
for (auto &th : threads) {
    th.join();
  }
  coeffSolved = coeff;
    return coeff;
}





Eigen::VectorXd QPpolyTraj::calculateCurrentPt( int Order, double time){
    bool lastPoint = false;
    Eigen::VectorXd current_pt_(dim);
    //Calculate the length of the segment inefficinet figure it out
//CONDITION CHECK THAT THERE WAS A SUCCESFUL WORK ON ALL 4 AXISES
    if (!(traj_valid[0]&&traj_valid[1]&&traj_valid[2] &&traj_valid[3])){
//IF any of the above is false
		std::cout << "FAILURE GENERATING PATH" <<std::endl;
		return Eigen::VectorXd::Zero(dim);
    }

    int Segment_idx =0;
    while (segmentTimes[Segment_idx] < time){
		time -= segmentTimes[Segment_idx];
		Segment_idx++;
		if (Segment_idx == segmentTimes.size()){
           lastPoint =true;
			break;
		}
    }
    if(lastPoint){
		Segment_idx=segmentTimes.size()-1;
		time = segmentTimes[Segment_idx];
     }
    //Normalize the time based on the fulltime
    //std::cout << "Updating power basis" << std::endl;
    Eigen::VectorXd power = basis(time,Order);
    double culsum = 0;
    for (int d = 0; d < dim; d++) {
		int count = 0;
		culsum = 0;
//std::cout << "Updating dimension: " << d << std::endl;
		for (int p = (polyOrder*Segment_idx); p < polyOrder*(Segment_idx+1); p++) {
			culsum = culsum + power(count) * coeffSolved(p, d);
			count = count +1;
		}
	//std::cout << "The sum is: " << culsum << std::endl;
		current_pt_(d) = culsum;
	//std::cout << "The current_pt_(d) is: " << current_pt_ << std::endl;
	}
    return current_pt_;
}


Eigen::MatrixXd QPpolyTraj::evalTraj(double time){	
    int numSeg = segmentTimes.size();
	//Calculates the fulltimes of each segment
	//Example if you have 2 segmes 1,0.5
	//Fultime would be 0, 1, 1.5
	//Calculate the length of the segment inefficinet figure it out
	//CONDITION CHECK THAT THERE WAS A SUCCESFUL WORK ON ALL 4 AXISES
	if (!(traj_valid[0]&&traj_valid[1]&&traj_valid[2] &&traj_valid[3])){
		//IF any of the above is false
		std::cout << "FAILURE GENERATING PATH" <<std::endl;
		return Eigen::MatrixXd::Constant(5,dim,-1e10);
	}
	int k=0;
	if(segmentTimes[numSeg-1] < k){
		return Eigen::MatrixXd::Constant(5,dim,-1e10);
	}
    while(segmentTimes[k] < time) {
		time -= segmentTimes[k];
		k+=1;
    }
	//std::cout << "segment : " << k << std::endl;
	Eigen::MatrixXd point(5,dim);
	for(int Order=0; Order < 5;Order++){
		//Normalize the time based on the fulltime
		Eigen::VectorXd power = basis(time,Order);
		for (int d = 0; d < dim; d++) {
			int count = 0;
			double culsum = 0;
			for (int p = (polyOrder*k); p < polyOrder*(k+1); p++) {
				culsum = culsum + power(count) * coeffSolved(p,d);
				count = count +1;
				}
			point(Order,d)= culsum;
		}
	}	
	return point;
}



bool QPpolyTraj::calcAngVBound(int segNum, float bounds){
	int derivOrder = 3;
	float accel = 5;
	Eigen::VectorXd temp_V;
	Eigen::VectorXd ConPoly = Eigen::VectorXd::Zero(2*(polyOrder-derivOrder)-1);
	for(int d = 0; d<4;d++){
		Eigen::VectorXd temp_dim =Eigen::VectorXd::Zero(polyOrder-derivOrder) ;
		if(d!=3){
			derivOrder=3;
		}
		else{
			derivOrder=1;
		}
		for(int i =derivOrder; i < polyOrder;i++){
			int factor = 1;
			for (int j = i; j > i-derivOrder; j--) {
				factor = factor*j;
			 }
			temp_dim(polyOrder-i-1) = factor*coeffSolved(i+segNum*polyOrder,d);
		}
		if(d!=3){
			temp_V = (2.5/(3*accel*accel))*rpoly_plus_plus::MultiplyPolynomials(temp_dim,temp_dim);
		}
		else{
			temp_V = (0.5)*rpoly_plus_plus::MultiplyPolynomials(temp_dim,temp_dim);
		}
		ConPoly = ConPoly + temp_V;
	}
	// Calculate a polynomial X^2+Y^2+Z^2 < V^2 -> 
	ConPoly(2*(polyOrder-derivOrder-1)) = ConPoly(2*(polyOrder-derivOrder-1)) - bounds*bounds;
//	std::cout << ConPoly <<std::endl; 
	//Evaluate at beginning of polynomial T=0;
	double val = rpoly_plus_plus::EvaluatePolynomial(ConPoly , 0);
	double endTime = segmentTimes[segNum];
	//Check the conditions on end points
	if(val > 0){
		return false;
	}
	val = rpoly_plus_plus::EvaluatePolynomial(ConPoly , endTime);
	if(val > 0){
		return false;
	}
	int numRoots = rpoly_plus_plus::FindSturmRoot(ConPoly,0,endTime);
	if(numRoots > 0){
		return false;
	}
	return true;
}




bool QPpolyTraj::calcThrRatBound(int segNum, float bounds){
	int derivOrder = 3;
	float factor = 1/(5*5);
	Eigen::VectorXd ConPoly = Eigen::VectorXd::Zero(2*(polyOrder-derivOrder)-1);
	for(int d = 0; d<3;d++){
		Eigen::VectorXd temp_dim =Eigen::VectorXd::Zero(polyOrder-derivOrder) ;
		for(int i =derivOrder; i < polyOrder;i++){
			//Calculate the constant i.e x^5 derivative twice is 5*4 x^3 
			int factor = 1;
			for (int j = i; j > i-derivOrder; j--) {
				factor = factor*j;
			 }
			temp_dim(polyOrder-i-1) = factor*coeffSolved(i+segNum*polyOrder,d);
		}
		Eigen::VectorXd temp_V = rpoly_plus_plus::MultiplyPolynomials(temp_dim,temp_dim);
		ConPoly = ConPoly + temp_V;
	}
	ConPoly(2*(polyOrder-derivOrder-1)) = ConPoly(2*(polyOrder-derivOrder-1)) - bounds*bounds;
	double val = rpoly_plus_plus::EvaluatePolynomial(ConPoly , 0);
	double endTime = segmentTimes[segNum];
	//Check the conditions on end points
	if(val > 0){
		return false;
	}
	val = rpoly_plus_plus::EvaluatePolynomial(ConPoly , endTime);
	if(val > 0){
		return false;
	}
	int numRoots = rpoly_plus_plus::FindSturmRoot(ConPoly,0,endTime);
	if(numRoots > 0){
		return false;
	}
	return true;
}



bool QPpolyTraj::calcThrBound(int segNum, float bounds){
	int derivOrder = 2;
	Eigen::VectorXd ConPoly = Eigen::VectorXd::Zero(2*(polyOrder-derivOrder)-1);
	for(int d = 0; d<3;d++){
		Eigen::VectorXd temp_dim =Eigen::VectorXd::Zero(polyOrder-derivOrder) ;
		for(int i =derivOrder; i < polyOrder;i++){
			//Calculate the constant i.e x^5 derivative twice is 5*4 x^3 
			int factor = 1;
			for (int j = i; j > i-derivOrder; j--) {
				factor = factor*j;
			 }
			 //std::cout <<factor <<std::endl;
			//Polynomial class is reversed highest order first lowest order last compared to ours.
			temp_dim(polyOrder-i-1) = factor*coeffSolved(i+segNum*polyOrder,d);
			//Last vlue gravity
			if((d==2)&&(i==polyOrder-1)){
				temp_dim(polyOrder-i-1) -=9.81;
			}
		}
		Eigen::VectorXd temp_V = rpoly_plus_plus::MultiplyPolynomials(temp_dim,temp_dim);
		ConPoly = ConPoly + temp_V;
	}
	ConPoly(2*(polyOrder-derivOrder-1)) = ConPoly(2*(polyOrder-derivOrder-1)) - bounds*bounds;
	double val = rpoly_plus_plus::EvaluatePolynomial(ConPoly , 0);
	double endTime = segmentTimes[segNum];
	if(val > 0){
		return false;
	}
	val = rpoly_plus_plus::EvaluatePolynomial(ConPoly , endTime);
	if(val > 0){
		return false;
	}
	int numRoots = rpoly_plus_plus::FindSturmRoot(ConPoly,0,endTime);
	if(numRoots > 0){
		return false;
	}
	return true;
}


bool QPpolyTraj::calcGlobalBound(int segNum, int derivOrder){
	if(limits[derivOrder]==0){
		return true;
	}
	Eigen::VectorXd ConPoly = Eigen::VectorXd::Zero(2*(polyOrder-derivOrder)-1);
	for(int d = 0; d<3;d++){
		Eigen::VectorXd temp_dim =Eigen::VectorXd::Zero(polyOrder-derivOrder) ;
		for(int i =derivOrder; i < polyOrder;i++){
			//Calculate the constant i.e x^5 derivative twice is 5*4 x^3 
			int factor = 1;
			for (int j = i; j > i-derivOrder; j--) {
				factor = factor*j;
			 }
			 //std::cout <<factor <<std::endl;
			//Polynomial class is reversed highest order first lowest order last compared to ours.
			temp_dim(polyOrder-i-1) = factor*coeffSolved(i+segNum*polyOrder,d);
		}
		//std::cout << temp_dim << std::endl;
		//if((derivOrder ==2)&&(d==2)){
		//	temp_dim(polyOrder-derivOrder-1)-=9.81;
		//}
		Eigen::VectorXd temp_V = rpoly_plus_plus::MultiplyPolynomials(temp_dim,temp_dim);
		//std::cout << "Dimension : " << d << std::endl;
		ConPoly = ConPoly + temp_V;
	}
	int bounds = limits[derivOrder]*limits[derivOrder];
	// Calculate a polynomial X^2+Y^2+Z^2 < V^2 -> 
	ConPoly(2*(polyOrder-derivOrder-1)) = ConPoly(2*(polyOrder-derivOrder-1)) - bounds;
//	std::cout << ConPoly <<std::endl; 
	//Evaluate at beginning of polynomial T=0;
	double val = rpoly_plus_plus::EvaluatePolynomial(ConPoly , 0);
	double endTime = segmentTimes[segNum];
	//Check the conditions on end points
	if(val > 0){
		return false;
	}
	val = rpoly_plus_plus::EvaluatePolynomial(ConPoly , endTime);
	if(val > 0){
		return false;
	}
	int numRoots = rpoly_plus_plus::FindSturmRoot(ConPoly,0,endTime);
	if(numRoots > 0){
		return false;
	}
	return true;
}

Eigen::MatrixXd QPpolyTraj::generateQ(int minDeriv, double time) {
    Eigen::MatrixXd Q( polyOrder,  polyOrder);
    Eigen::VectorXd poly =  basis(time, minDeriv);
    double powerOrder = 0.0;
    for (int i = 0; i <  polyOrder; i++) {
        for (int j = 0; j <  polyOrder; j++) {
            powerOrder = ((double)j + (double)i) - ((double)minDeriv * 2) + 1.0;
            if (powerOrder < 1) {
                powerOrder = 1;
            }
            Q(i, j) = poly(j) * poly(i) * time / powerOrder;
        }
    }
    return Q;
}


Eigen::MatrixXd QPpolyTraj::generateObjFun(int minDeriv)
{
    int numberSegments = segmentTimes.size();
    //Eigen::MatrixXd startQ= Eigen::MatrixXd::Zero( polyOrder,  polyOrder);
    Eigen::MatrixXd endQ( polyOrder,  polyOrder);
    Eigen::MatrixXd Qtotal = Eigen::MatrixXd::Zero(polyOrder*numberSegments, polyOrder * numberSegments);
    for (int i = 0; i < numberSegments; i++) {
        endQ = generateQ(minDeriv, segmentTimes[i]);
        //COPY Matrix Code
        Qtotal.block(i *  polyOrder, i *  polyOrder,polyOrder,  polyOrder) = endQ ;
        //startQ = endQ;
    }
    return Qtotal;
}


QP_ineq_const QPpolyTraj::genInEqConstraint( int dimension)
{
	double dt = 0.01; // Make a class member
	int numConst =0;
    int coeffNum = (vertices.size() - 1) *  polyOrder;
	QP_ineq_const  ineq_const;
	for (int i = 1; i < vertices.size();i++){
		//Count the number of inequality constraints you have
		for(int j =0; j < vertices[i].ineq_constraint.size(); j++){
			double toff = vertices[i].ineq_constraint[j].timeOffset;
			numConst += (vertices[i].ineq_constraint[j].InEqDim(dimension)*toff/dt+1);
		}
	}
	int add_constr_num = 0;
	QP_ineq_const temp_ineq_constr;
	if(add_ineq_constr.size()!=0){
		temp_ineq_constr = add_ineq_constr[dimension];
		add_constr_num = temp_ineq_constr.d.rows();
		numConst +=add_constr_num;
	}

	//no inequality constraint just send the inequality constraint empty
	// Empty means that the QP optimization has no ineq_const
	//std::cout <<"num constraints: " << numConst << std::endl;
	if(numConst==0){
		ineq_const.d= Eigen::VectorXd::Zero(1);
		ineq_const.d(0) = -0.1;
		ineq_const.f= Eigen::VectorXd::Zero(1);
		ineq_const.f(0) = 0.1;
		ineq_const.C =  Eigen::MatrixXd::Zero(1, coeffNum);
		return ineq_const;
	}
	ineq_const.d= Eigen::VectorXd::Zero(numConst);
	ineq_const.f= Eigen::VectorXd::Zero(numConst);
	ineq_const.C =  Eigen::MatrixXd::Zero(numConst, coeffNum);
	int rowNum = 0;
	for(int i = 1; i < vertices.size(); i++){
		for(int j =0; j < vertices[i].ineq_constraint.size(); j++){// Check if it is a valid constraint
			if(vertices[i].ineq_constraint[j].InEqDim(dimension)==1){
				double time = segmentTimes[i-1]; //modify to be any waypoints 
				waypoint_ineq_const pon_ineq = vertices[i].ineq_constraint[j];
				double toff = time - pon_ineq.timeOffset;
				//std::cout << "End time " << time << std::endl;
				while(toff < time){
					//std::cout << "TIME OF CONSTRAINT " << toff << std::endl;
					Eigen::VectorXd row = basis(toff, pon_ineq.derivOrder);
					ineq_const.d(rowNum) = pon_ineq.lower(dimension);
					ineq_const.f(rowNum) = pon_ineq.upper(dimension);
					ineq_const.C.block(rowNum, (i -1)*  polyOrder,1, polyOrder) = row.transpose(); 
					rowNum +=1;
					toff+=dt;
				}	
			}
		}
	}
	
	if(add_ineq_constr.size()!=0){

		ineq_const.d.tail(add_constr_num) = temp_ineq_constr.d;
		ineq_const.C.block(rowNum, 0,add_constr_num, coeffNum) = temp_ineq_constr.C;
		ineq_const.f.tail(add_constr_num) = temp_ineq_constr.f;
	}
	return ineq_const ;
}


QP_constraint QPpolyTraj::genConstraint( int dimension, int numConstraint)
{
    //do the fixed first
    //We have  polyOrder coefficient polynomials
   //#As a result, we need 20 coefficients for 2 segments.
	int add_constr_num = 0;
	QP_constraint temp_constr;
	if(add_eq_constr.size()!=0){
		temp_constr = add_eq_constr[dimension];
		add_constr_num = temp_constr.a.rows();
		numConstraint +=add_constr_num;
	}
    int coeffNum = (vertices.size() - 1) *  polyOrder;
    //#Constant conditions go through each way point and start/end with no velocity/acceleration
    Eigen::VectorXd row;
    Eigen::VectorXd point;
	//Keeps track if you fixed the derivative at the point or not
    Eigen::MatrixXd fixedDeriv = Eigen::MatrixXd::Zero(segmentTimes.size(), 5);
	Eigen::MatrixXd b = Eigen::VectorXd::Zero(numConstraint);
	Eigen::MatrixXd A = Eigen::MatrixXd::Zero(numConstraint, coeffNum); //create the equality constraint 
    double time = 0;
    int rowindex = 0;
	if (numConstraint > coeffNum){
		QP_constraint qp;
		qp.a = A;
		qp.b = b;
		return qp;
	}
    for (int i = 0; i < vertices.size(); i++) {
        if (i == 0) {
            //We start at time 0 make sure your first possition is correct
            row = basis( time, 0);
            A.block(rowindex, i *  polyOrder,1, polyOrder) = row.transpose();
            vertices[i].getPos(&point);
            b(rowindex) =point(dimension);
            rowindex++;
            //Make sure that all the velocity acceleration snap jerk are 0
            for (int j = 1; j < 5; j++) {
				//Check if there is a constraint here
				if(vertices[i].getConstraint(&point,j)==1){
					// if true set the dimension
					//std::cout << "axes: " << dimension << std::endl;
					//std::cout << "DerivOrder: " << j << std::endl;
					//std::cout << "Vertex i:  " << i << std::endl;
					//std::cout << "Constraint: " << point << std::endl;
					b(rowindex) =point(dimension);//*normalFactor;
				}
				else{
						b(rowindex) = 0;
				}
				row =  basis( time,j);
				A.block(rowindex, i *  polyOrder,1, polyOrder) = row.transpose();
				rowindex++;
            }
        }
        if (i == (vertices.size() - 1)) {
			//We start at the end time make sure your possition is correct
	        time = segmentTimes[segmentTimes.size() - 1];
            row =  basis( time,0);
            A.block(rowindex,( i-1) *  polyOrder,1,  polyOrder) = row.transpose();
            vertices[i].getPos(&point);
            b(rowindex) = point(dimension);
            rowindex++;	
	        //Make sure that all the velocity acceleration snap jerk are 0
			for (int j = 1; j < 5; j++) {
				row =  basis( time,j);
				if(vertices[i].getConstraint(&point,j)==1){
					A.block(rowindex, (i - 1) *  polyOrder,1,  polyOrder) =  row.transpose();
					// if true set the dimension
					//std::cout << "Dimension constraint i: " << j <<std::endl;
					b(rowindex) = point(dimension);
					//std::cout << "axes: " << dimension << std::endl;
				//	std::cout << "DerivOrder: " << j << std::endl;
					//std::cout << "Vertex i:  " << i << std::endl;
					//std::cout << "Constraint: " << point << std::endl;
					rowindex++;
				}
				//else{
				//	b(rowindex) = 0;
				//}
			}
        }
        if((i != 0)&&(i != (vertices.size() - 1))) {
            time = segmentTimes[i - 1];
            row =  basis(time,0);
            vertices[i].getPos(&point);
            //Make sure both the prev segment ends in the location
            //And the prev segment starts in the location
            A.block(rowindex, (i - 1) *  polyOrder,1,  polyOrder) = row.transpose();
            b(rowindex) = point(dimension);
            rowindex++;
            //Making sure the next segment starts in the right area.
            row =  basis(0,0);
            A.block(rowindex, i  *  polyOrder,1,  polyOrder) = row.transpose();
            b(rowindex) = point(dimension);
            rowindex++;
			//Check to see if we have set constraints at each waypoint aside from position
            for (int j = 1; j < 5; j++) {
				if(vertices[i].getConstraint(&point,j)==1){
					row =  basis( time,j);
					// if true set the dimension at the order
					A.block(rowindex, (i - 1) *  polyOrder,1,  polyOrder) =  row.transpose();
					b(rowindex) = point(dimension);
					rowindex++;
					//Making sure the next segment starts in the right area.
					row =  basis(0,j);
					A.block(rowindex, i  *  polyOrder,1,  polyOrder) =row.transpose();
					b(rowindex) = point(dimension);
					rowindex++;
					// sets a flag to let us know we can skip the bottom continuity 
					fixedDeriv(i,j) = 1;
					/*
					std::cout << "axes: " << dimension << std::endl;
					std::cout << "DerivOrder: " << j << std::endl;
					std::cout << "Vertex i:  " << i << std::endl;
					std::cout << "Constraint: " << point << std::endl;*/
				}
            }
        }
    }
    //Continuous constraints i.e. ensure contiunity of velocity and acceleration.
    for (int i = 1; i < segmentTimes.size(); i++) {
        for (int j = 1; j < 5; j++) {
			// You only need to enfroce continuity constraints if at waypoint N a number isn't set
			//i.e at point 1 segment 0 velocity = 1 then segment 1 velcoity also = 1, and as a result 
			//s0v = 1 ;s1v =1;  =>s0v = s1v; this is no longer needed. to expliclitly state it
			if (fixedDeriv(i,j)==0){
				//endpoint of last segment
				row =  basis( segmentTimes[i - 1], j);
				A.block(rowindex, (i-1) *  polyOrder,1,  polyOrder) = row.transpose();
				//Start of Current Segment. 
				row =  basis( 0, j);
				A.block(rowindex, i *  polyOrder,1,  polyOrder) = -1*row.transpose();
				b(rowindex) = 0;
				rowindex++;
			}
        }
    }
	if(add_eq_constr.size()!=0){
		b.block(rowindex, 0,add_constr_num, 1) = temp_constr.b;
		A.block(rowindex, 0,add_constr_num, coeffNum) = temp_constr.a;
	}

	QP_constraint qp;
	qp.a = A;
	qp.b = b;
	//std::cout << A <<std::endl;
	//std::cout << b <<std::endl;
	//Eigen::MatrixXd A_squared = A* A.transpose();
	//std::cout << "Condition Number " << A_squared.norm()*A_squared.inverse().norm() << std::endl;
	return qp;
}


Eigen::MatrixXd QPpolyTraj::generateJointObjFun(int minDeriv){
   	Eigen::MatrixXd D = generateObjFun(minDeriv);
    int coeffNum = (vertices.size() - 1) *  polyOrder;
	Eigen::MatrixXd comp  = Eigen::MatrixXd::Zero(coeffNum*dim,coeffNum*dim);
	for (int j = 0; j < dim; j++){
		//Create Copy since the quadprog changes the X&T Q  X matrix each run
        comp.block(j *  coeffNum, j *  coeffNum,coeffNum,  coeffNum) = D;
	}
	return comp;
}

QP_constraint QPpolyTraj::genJointConstraint(){
	QP_constraint joint_qp;
	std::vector<int> numConstrDim;
    	int coeffNum = (vertices.size() - 1) *  polyOrder;
	int sumConstraint = 0;
	 //Don't have pure 0's on the derivative of the last object
	int numConstraint = 6 * vertices.size() - 2;
	//calculate the number of vertice constraint
	for (int i = 1; i <vertices.size()-1; i++){
		int addConstraint = vertices[i].getStatus().sum() -1;
		numConstraint +=addConstraint;
	}		
	for(int i=0;i<dim;i++){
		int add_constr=0;
		if(add_eq_constr.size()!=0){
			add_constr = add_eq_constr[i].b.rows();
		}
		numConstrDim.push_back(add_constr+numConstraint);
		sumConstraint += numConstrDim[i];
	}	
	sumConstraint+=add_joint_eq_constr.b.rows();
	Eigen::MatrixXd btotal = Eigen::VectorXd::Zero(sumConstraint );
	Eigen::MatrixXd Atotal  = Eigen::MatrixXd::Zero(sumConstraint , coeffNum*dim ); //create the equality constraint 
	int currentRow =0;
	for (int j = 0; j < dim; j++){
		QP_constraint qp = genConstraint(j,numConstraint ); //each dimension has its unique equality constraint
		Eigen::MatrixXd A = qp.a, b = qp.b;		
		Atotal.block(currentRow,coeffNum*j,numConstrDim[j], coeffNum) = A;
		btotal.block(currentRow,0,numConstrDim[j],1) = b;
		//std::cout << "Block dim inserted: " << j << std::endl;
		currentRow+=numConstrDim[j];
	}
	//Add the joint constraints
	//std::cout << "Number of rows a: " << add_joint_eq_constr.a.rows() <<std::endl;
	if(add_joint_eq_constr.a.rows() > 0){
		Atotal.block(currentRow,0,add_joint_eq_constr.b.rows(), coeffNum*dim) = add_joint_eq_constr.a;
		btotal.block(currentRow,0,add_joint_eq_constr.b.rows(),1) = add_joint_eq_constr.b;
	}
	joint_qp.a = Atotal;
	joint_qp.b = btotal;
	return joint_qp;
}

QP_ineq_const QPpolyTraj::genInEqJointConstraint(){
	QP_ineq_const  joint_ineq_const;
    int coeffNum = (vertices.size() - 1) *  polyOrder;
	std::vector<int> numConstrDim;
	int sumConstraint = 0;
	for(int k=0;k<dim;k++){
		int numConst = 0;
		for (int i = 1; i < vertices.size();i++){
			//Count the number of inequality constraints you have
			for(int j =0; j < vertices[i].ineq_constraint.size(); j++){
				double toff = vertices[i].ineq_constraint[j].timeOffset;
				numConst += (vertices[i].ineq_constraint[j].InEqDim(k)*toff/0.02+1);
			}
		}
		if(add_ineq_constr.size()!=0){
			numConst +=add_ineq_constr[k].d.rows();
		}
		numConstrDim.push_back(numConst) ;
		sumConstraint += numConstrDim[k];
	}	
	sumConstraint+=add_joint_ineq_constr.d.rows();	
	if(sumConstraint ==0){
		joint_ineq_const.d= Eigen::VectorXd::Zero(1);
		joint_ineq_const.d(0) = -0.1;
		joint_ineq_const.f= Eigen::VectorXd::Zero(1);
		joint_ineq_const.f(0) = 0.1;
		joint_ineq_const.C =  Eigen::MatrixXd::Zero(1, coeffNum*dim);
		return joint_ineq_const;		
	}
	joint_ineq_const.C = Eigen::MatrixXd::Zero(sumConstraint , coeffNum*dim );
	joint_ineq_const.d = Eigen::VectorXd::Zero(sumConstraint);
	joint_ineq_const.f = Eigen::VectorXd::Zero(sumConstraint);	
	int currentRow =0;
	for (int j = 0; j < dim; j++){
		if(numConstrDim[j]!=0){
			QP_ineq_const temp_ineq_qp = genInEqConstraint(j ); //each dimension has its unique equality constraint
			joint_ineq_const.C.block(currentRow,coeffNum*j,numConstrDim[j], coeffNum) = temp_ineq_qp.C;
			joint_ineq_const.d.block(currentRow,0,numConstrDim[j],1) = temp_ineq_qp.d;
			joint_ineq_const.f.block(currentRow,0,numConstrDim[j],1) = temp_ineq_qp.f;
			currentRow+=numConstrDim[j];
		}
	}
	if(add_joint_ineq_constr.d.rows()!=0){
		//Add the joint constraints
		joint_ineq_const.C.block(currentRow,0,add_joint_ineq_constr.d.rows(), coeffNum*dim) = add_joint_ineq_constr.C;
		joint_ineq_const.d.block(currentRow,0,add_joint_ineq_constr.d.rows(),1) = add_joint_ineq_constr.d;
		joint_ineq_const.f.block(currentRow,0,add_joint_ineq_constr.d.rows(),1) = add_joint_ineq_constr.f;
	}
	return joint_ineq_const;
}

Eigen::MatrixXd QPpolyTraj::calculateTrajectory( int Order, double dt){	
    int numSeg = segmentTimes.size();
	//Calculates the fulltimes of each segment
	//Example if you have 2 segmes 1,0.5
	//Fultime would be 0, 1, 1.5
	Eigen::VectorXd fullTime(numSeg+1);
	//Calculate the length of the segment inefficinet figure it out
	int length = 0;
    for (int k = 0; k < segmentTimes.size(); k++) {
        for (int j = 0; j < segmentTimes[k]  /0.01; j++) {
			length++;				
		}	
    }
    int startTime = 0;
	//CONDITION CHECK THAT THERE WAS A SUCCESFUL WORK ON ALL 4 AXISES
	if (!(traj_valid[0]&&traj_valid[1]&&traj_valid[2] &&traj_valid[3])){
		//IF any of the above is false
		std::cout << "FAILURE GENERATING PATH" <<std::endl;
		return Eigen::MatrixXd::Zero(length,dim);
	}
	Eigen::MatrixXd trajectory(length,dim);
	int rownum = 0;
	double culsum = 0;
    for (int k = 0; k < segmentTimes.size(); k++) {
        for (int j = 0; j < segmentTimes[k]  /dt; j++) {
			double time = j*dt;
			//Normalize the time based on the fulltime
			Eigen::VectorXd power = basis(time,Order);
			for (int d = 0; d < dim; d++) {
				int count = 0;
				culsum = 0;
				for (int p = (polyOrder*k); p < polyOrder*(k+1); p++) {
					culsum = culsum + power(count) * coeffSolved(p,d);
					count = count +1;
					}
				trajectory(rownum,d)= culsum;
			}
			rownum++;				
		}	
    }
	return trajectory;
}

Eigen::VectorXd QPpolyTraj::basis(double time, int derivative) {
    if (derivative > 4) {
        throw 10;
    }
    // compute all the powers of time
    std::vector<double> t(polyOrder);
	//create a list of powers temporarily so you don't have to recalculate each time
    for (int i = 0; i <polyOrder; i++) {
        t[i] = pow(time, i);
    }
	// Alocate coefficitions of your basis
    Eigen::VectorXd b(polyOrder);
	// Calculate coefficients 
    for (int i = 0; i < polyOrder; i++) {
		double power = i - derivative;
		double coeff = 1;
		//if your power < derivative it must be 0 i.e. x'' = 0; or x^2''' = 0;
		if (power <0){
			b[i] = 0;
		}
		else{
			//claculate n!/(n-derivative)!
			 for (int j = i; j > i-derivative; j--) {
				coeff = coeff*j;
			 }
			b[i] = coeff*t[power];
		}
    }
    return b;
}
