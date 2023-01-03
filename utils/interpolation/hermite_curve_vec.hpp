#pragma once

#include "hermite_curve.hpp"
#include <memory>
#include <Eigen/Dense>


class HermiteCurveVec {
public:
  HermiteCurveVec();

  HermiteCurveVec(const unsigned int dim);

  ~HermiteCurveVec();

  void initialize(const unsigned int dim);

  void add_curve(std::unique_ptr<HermiteCurve> spline);
  Eigen::VectorXd evaluate(const double &t_in);
  Eigen::VectorXd evaluateFirstDerivative(const double &t_in);
  Eigen::VectorXd evaluateSecondDerivative(const double &t_in);

protected:
  unsigned int dim_;
  Eigen::VectorXd output;
  std::vector<std::shared_ptr<HermiteCurve>> curves;
};
