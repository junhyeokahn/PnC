#ifndef ALM_HERMITE_CURVE_VEC_H
#define ALM_HERMITE_CURVE_VEC_H

#include <Eigen/Dense>
#include <Utils/Math/hermite_curve.hpp>

// vector version of hermite curve interpolation

class HermiteCurveVec{
public:
	HermiteCurveVec();
	HermiteCurveVec(const Eigen::VectorXd & start_pos, const Eigen::VectorXd & start_vel, 
				   const Eigen::VectorXd & end_pos, const Eigen::VectorXd & end_vel);
	~HermiteCurveVec();
	Eigen::VectorXd evaluate(const double & s_in);
	Eigen::VectorXd evaluateFirstDerivative(const double & s_in);
	Eigen::VectorXd evaluateSecondDerivative(const double & s_in);

private:
	Eigen::VectorXd p1;
	Eigen::VectorXd v1;
	Eigen::VectorXd p2;
	Eigen::VectorXd v2;

	std::vector<HermiteCurve> curves;
 	Eigen::VectorXd output;
};

#endif