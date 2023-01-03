#include "hermite_curve_vec.hpp"

HermiteCurveVec::HermiteCurveVec() {  curves.clear();}

HermiteCurveVec::HermiteCurveVec(const unsigned int dim) {
  // Clear and 	create N hermite curves with the specified boundary conditions
  dim_ = dim;
  curves.clear();
  output = Eigen::VectorXd::Zero(dim);
}

HermiteCurveVec::~HermiteCurveVec() { curves.clear();}

void HermiteCurveVec::initialize(const unsigned int dim) {
  // Clear and 	create N hermite curves with the specified boundary conditions
  dim_ = dim;
  curves.clear();
  output = Eigen::VectorXd::Zero(dim);
}

void HermiteCurveVec::add_curve(std::unique_ptr<HermiteCurve> spline) {
  curves.push_back(std::move(spline));
}

Eigen::VectorXd HermiteCurveVec::evaluate(const double &t_in) {
  for (int i = 0; i < dim_; i++) {
    output[i] = curves[i]->evaluate(t_in);
  }
  return output;
}

Eigen::VectorXd HermiteCurveVec::evaluateFirstDerivative(const double &t_in) {
  for (int i = 0; i < dim_; i++) {
    output[i] = curves[i]->evaluateFirstDerivative(t_in);
  }
  return output;
}

Eigen::VectorXd HermiteCurveVec::evaluateSecondDerivative(const double &t_in) {
  for (int i = 0; i < dim_; i++) {
    output[i] = curves[i]->evaluateSecondDerivative(t_in);
  }
  return output;
}
