#ifndef MIN_JERK_CURVE_VEC_H
#define MIN_JERK_CURVE_VEC_H

#include <Eigen/Dense>
#include <Utils/Math/minjerk_one_dim.hpp>

// vector version of min jerk interpolation

class MinJerkCurveVec{
public:
	MinJerkCurveVec();
	MinJerkCurveVec(const Eigen::VectorXd & start_pos, const Eigen::VectorXd & start_vel, const Eigen::VectorXd & start_acc, 
			  	    const Eigen::VectorXd & end_pos, const Eigen::VectorXd & end_vel, const Eigen::VectorXd & end_acc);
	~MinJerkCurveVec();
	Eigen::VectorXd evaluate(const double & s_in);
	Eigen::VectorXd evaluateFirstDerivative(const double & s_in);
	Eigen::VectorXd evaluateSecondDerivative(const double & s_in);

private:
	Eigen::VectorXd p1;
	Eigen::VectorXd v1;
	Eigen::VectorXd a1;
	
	Eigen::VectorXd p2;
	Eigen::VectorXd v2;
	Eigen::VectorXd a2;

	std::vector<MinJerk_OneDimension> curves;
 	Eigen::VectorXd output;
};

#endif