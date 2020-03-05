#include <Utils/Math/minjerk_vec.hpp>

// Constructor
MinJerkCurveVec::MinJerkCurveVec(){}


MinJerkCurveVec::MinJerkCurveVec(const Eigen::VectorXd & start_pos, const Eigen::VectorXd & start_vel, const Eigen::VectorXd & start_acc, 
			  	    const Eigen::VectorXd & end_pos, const Eigen::VectorXd & end_vel, const Eigen::VectorXd & end_acc):
					p1(start_pos), v1(start_vel), a1(start_acc), 
					p2(end_pos), v2(end_vel), a2(end_acc){

	// Create N minjerk curves with the specified boundary conditions
	for(int i = 0; i < start_pos.size(); i++){
		curves.push_back( MinJerk_OneDimension(Eigen::Vector3d(p1[i], v1[i], a1[i]),
											   Eigen::Vector3d(p2[i], v2[i], a2[i]),
											   0.0, 1.0));
	}
	output = Eigen::VectorXd::Zero(start_pos.size());
}

// Destructor
MinJerkCurveVec::~MinJerkCurveVec(){}

// Evaluation functions
Eigen::VectorXd MinJerkCurveVec::evaluate(const double & s_in){
	double val;
	for(int i = 0; i < p1.size(); i++){
		curves[i].getPos(s_in, val);
		output[i] = val;
	}
	return output;
}

Eigen::VectorXd MinJerkCurveVec::evaluateFirstDerivative(const double & s_in){
	double val;
	for(int i = 0; i < v1.size(); i++){
		curves[i].getVel(s_in, val);
		output[i] = val;
	}
	return output;
}

Eigen::VectorXd MinJerkCurveVec::evaluateSecondDerivative(const double & s_in){
	double val;
	for(int i = 0; i < a1.size(); i++){
		curves[i].getAcc(s_in, val);
		output[i] = val;
	}
	return output;
}
