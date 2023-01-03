#include "interpolation.hpp"
#include "cubic_hermite_curve.hpp"

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "configuration.hpp"

namespace util {
double Smooth(double ini, double fin, double rat) {
  double ret(0.);
  if (rat < 0) {
    return ini;
  } else if (rat > 1) {
    return fin;
  } else {
    return ini + (fin - ini) * rat;
  }
}

double SmoothPos(double ini, double end, double moving_duration,
                 double curr_time) {
  double ret;
  ret = ini + (end - ini) * 0.5 * (1 - cos(curr_time / moving_duration * M_PI));
  if (curr_time > moving_duration) {
    ret = end;
  }
  return ret;
}

double SmoothVel(double ini, double end, double moving_duration,
                 double curr_time) {
  double ret;
  ret = (end - ini) * 0.5 * (M_PI / moving_duration) *
        sin(curr_time / moving_duration * M_PI);
  if (curr_time > moving_duration) {
    ret = 0.0;
  }
  return ret;
}
double SmoothAcc(double ini, double end, double moving_duration,
                 double curr_time) {
  double ret;
  ret = (end - ini) * 0.5 * (M_PI / moving_duration) *
        (M_PI / moving_duration) * cos(curr_time / moving_duration * M_PI);
  if (curr_time > moving_duration) {
    ret = 0.0;
  }
  return ret;
}

void SinusoidTrajectory(double initTime_, const Eigen::VectorXd &midPoint_,
                        const Eigen::VectorXd &amp_,
                        const Eigen::VectorXd &freq_, double evalTime_,
                        Eigen::VectorXd &p_, Eigen::VectorXd &v_,
                        Eigen::VectorXd &a_, double smoothing_dur) {
  int dim = midPoint_.size();
  p_ = Eigen::VectorXd::Zero(dim);
  v_ = Eigen::VectorXd::Zero(dim);
  a_ = Eigen::VectorXd::Zero(dim);
  for (int i = 0; i < dim; ++i) {
    p_[i] = amp_[i] * sin(2 * M_PI * freq_[i] * (evalTime_ - initTime_)) +
            midPoint_[i];
    v_[i] = amp_[i] * 2 * M_PI * freq_[i] *
            cos(2 * M_PI * freq_[i] * (evalTime_ - initTime_));
    a_[i] = -amp_[i] * 2 * M_PI * freq_[i] * 2 * M_PI * freq_[i] *
            sin(2 * M_PI * freq_[i] * (evalTime_ - initTime_));
  }
  if (evalTime_ < (initTime_ + smoothing_dur)) {
    double s = SmoothPos(0., 1., smoothing_dur, evalTime_ - initTime_);
    for (int i = 0; i < dim; ++i) {
      v_[i] *= s;
      a_[i] *= s;
    }
  }
}
} // namespace util

HermiteQuaternionCurve::HermiteQuaternionCurve() {}

HermiteQuaternionCurve::HermiteQuaternionCurve(
    const Eigen::Quaterniond &quat_start,
    const Eigen::Vector3d &angular_velocity_start,
    const Eigen::Quaterniond &quat_end,
    const Eigen::Vector3d &angular_velocity_end, double duration) {
  initialize(quat_start, angular_velocity_start, quat_end, angular_velocity_end,
             duration);
}

void HermiteQuaternionCurve::initialize(
    const Eigen::Quaterniond &quat_start,
    const Eigen::Vector3d &angular_velocity_start,
    const Eigen::Quaterniond &quat_end,
    const Eigen::Vector3d &angular_velocity_end, double duration) {
  qa = quat_start;
  omega_a = angular_velocity_start;

  qb = quat_end;
  omega_b = angular_velocity_end;

  t_dur = duration;

  initialize_data_structures();
}

HermiteQuaternionCurve::~HermiteQuaternionCurve() {}

void HermiteQuaternionCurve::initialize_data_structures() {
  //
  // q(t) = exp( theta(t) ) * qa : global frame
  // q(t) = qa * exp( theta(t) ) : local frame
  // where theta(t) is hermite cubic spline with
  // theta(0) = 0, theta(t_dur) = log(delq_ab)
  // dot_theta(0) = omega_a, dot_theta(1) = omega_b

  Eigen::AngleAxisd delq_ab = Eigen::AngleAxisd(qb * qa.inverse());
  // Eigen::AngleAxisd del_qab = qa.inverse()*qb;

  Eigen::VectorXd start_pos = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd start_vel = omega_a;
  Eigen::VectorXd end_pos = delq_ab.axis() * delq_ab.angle();
  Eigen::VectorXd end_vel = omega_b;

  theta_ab.initialize(start_pos.size());
  for (int i = 0; i < start_pos.size(); i++) {
    theta_ab.add_curve(std::make_unique<CubicHermiteCurve>(
            start_pos[i], start_vel[i], end_pos[i], end_vel[i], t_dur));
  }

}

void HermiteQuaternionCurve::evaluate(const double &t_in,
                                      Eigen::Quaterniond &quat_out) {
  Eigen::VectorXd delq_vec = theta_ab.evaluate(t_in);

  if (delq_vec.norm() < 1e-6)
    delq = Eigen::Quaterniond(1, 0, 0, 0);
  else
    delq = Eigen::AngleAxisd(delq_vec.norm(), delq_vec / delq_vec.norm());
  // quat_out = q0 * delq; // local frame
  quat_out = delq * qa; // global frame
}

void HermiteQuaternionCurve::getAngularVelocity(const double &t_in,
                                                Eigen::Vector3d &ang_vel_out) {
  ang_vel_out = theta_ab.evaluateFirstDerivative(t_in);
}

// For world frame
void HermiteQuaternionCurve::getAngularAcceleration(
    const double &t_in, Eigen::Vector3d &ang_acc_out) {
  ang_acc_out = theta_ab.evaluateSecondDerivative(t_in);
  // not sure about this
}

void HermiteQuaternionCurve::printQuat(const Eigen::Quaterniond &quat) {
  std::cout << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w()
            << " " << std::endl;
}


