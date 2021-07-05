#include "utils/interpolation.hpp"

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <configuration.hpp>

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
                        Eigen::VectorXd &a_) {
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
  double smoothing_dur(1.0);
  if (evalTime_ < (initTime_ + smoothing_dur)) {
    for (int i = 0; i < dim; ++i) {
      v_[i] = 0. + v_[i] * (evalTime_ - initTime_) / smoothing_dur;
      a_[i] = 0. + a_[i] * (evalTime_ - initTime_) / smoothing_dur;
    }
  }
}
} // namespace util

HermiteCurve::HermiteCurve() {
  p1_ = 0;
  v1_ = 0;
  p2_ = 0;
  v2_ = 0;
  s_ = 0;
}

HermiteCurve::HermiteCurve(const double &start_pos, const double &start_vel,
                           const double &end_pos, const double &end_vel)
    : p1_(start_pos), v1_(start_vel), p2_(end_pos), v2_(end_vel) {
  s_ = 0;
}

HermiteCurve::~HermiteCurve() {}

void HermiteCurve::Initialize(const double &start_pos, const double &start_vel,
                              const double &end_pos, const double &end_vel) {
  p1_ = start_pos;
  p2_ = end_pos;
  v1_ = start_vel;
  v2_ = end_vel;
}

double HermiteCurve::Evaluate(const double &s_in) {
  s_ = util::Clamp(s_in, 0., 1.);
  return p1_ * (2 * std::pow(s_, 3) - 3 * std::pow(s_, 2) + 1) +
         p2_ * (-2 * std::pow(s_, 3) + 3 * std::pow(s_, 2)) +
         v1_ * (std::pow(s_, 3) - 2 * std::pow(s_, 2) + s_) +
         v2_ * (std::pow(s_, 3) - std::pow(s_, 2));
}

double HermiteCurve::EvaluateFirstDerivative(const double &s_in) {
  s_ = util::Clamp(s_in, 0., 1.);
  return p1_ * (6 * std::pow(s_, 2) - 6 * s_) +
         p2_ * (-6 * std::pow(s_, 2) + 6 * s_) +
         v1_ * (3 * std::pow(s_, 2) - 4 * s_ + 1) +
         v2_ * (3 * std::pow(s_, 2) - 2 * s_);
}

double HermiteCurve::EvaluateSecondDerivative(const double &s_in) {
  s_ = util::Clamp(s_in);
  return p1_ * (12 * s_ - 6) + p2_ * (-12 * s_ + 6) + v1_ * (6 * s_ - 4) +
         v2_ * (6 * s_ - 2);
}

HermiteCurveVec::HermiteCurveVec() {}

HermiteCurveVec::HermiteCurveVec(const Eigen::VectorXd &start_pos,
                                 const Eigen::VectorXd &start_vel,
                                 const Eigen::VectorXd &end_pos,
                                 const Eigen::VectorXd &end_vel)
    : p1_(start_pos), v1_(start_vel), p2_(end_pos), v2_(end_vel) {
  Initialize(start_pos, start_vel, end_pos, end_vel);
}

HermiteCurveVec::~HermiteCurveVec() {}

void HermiteCurveVec::Initialize(const Eigen::VectorXd &start_pos,
                                 const Eigen::VectorXd &start_vel,
                                 const Eigen::VectorXd &end_pos,
                                 const Eigen::VectorXd &end_vel) {
  // Clear and  create N hermite curves_ with the specified boundary conditions
  curves_.clear();
  p1_ = start_pos;
  v1_ = start_vel;
  p2_ = end_pos;
  v2_ = end_vel;

  for (int i = 0; i < start_pos.size(); i++) {
    curves_.push_back(
        HermiteCurve(start_pos[i], start_vel[i], end_pos[i], end_vel[i]));
  }
  output_ = Eigen::VectorXd::Zero(start_pos.size());
}

// Evaluation functions
Eigen::VectorXd HermiteCurveVec::Evaluate(const double &s_in) {
  for (int i = 0; i < p1_.size(); i++) {
    output_[i] = curves_[i].Evaluate(s_in);
  }
  return output_;
}

Eigen::VectorXd HermiteCurveVec::EvaluateFirstDerivative(const double &s_in) {
  for (int i = 0; i < p1_.size(); i++) {
    output_[i] = curves_[i].EvaluateFirstDerivative(s_in);
  }
  return output_;
}

Eigen::VectorXd HermiteCurveVec::EvaluateSecondDerivative(const double &s_in) {
  for (int i = 0; i < p1_.size(); i++) {
    output_[i] = curves_[i].EvaluateSecondDerivative(s_in);
  }
  return output_;
}

HermiteQuaternionCurve::HermiteQuaternionCurve() {}

HermiteQuaternionCurve::HermiteQuaternionCurve(
    const Eigen::Quaterniond &quat_start,
    const Eigen::Vector3d &angular_velocity_start,
    const Eigen::Quaterniond &quat_end,
    const Eigen::Vector3d &angular_velocity_end) {
  Initialize(quat_start, angular_velocity_start, quat_end,
             angular_velocity_end);
}

void HermiteQuaternionCurve::Initialize(
    const Eigen::Quaterniond &quat_start,
    const Eigen::Vector3d &angular_velocity_start,
    const Eigen::Quaterniond &quat_end,
    const Eigen::Vector3d &angular_velocity_end) {
  qa = quat_start;
  omega_a = angular_velocity_start;

  qb = quat_end;
  omega_b = angular_velocity_end;

  s_ = 0.0;
  initialize_data_structures();
}

HermiteQuaternionCurve::~HermiteQuaternionCurve() {}

void HermiteQuaternionCurve::initialize_data_structures() {
  q0 = qa;

  if (omega_a.norm() < 1e-6) {
    q1 = qa * Eigen::Quaterniond(1, 0, 0, 0);
  } else {
    q1 = qa * Eigen::Quaterniond(Eigen::AngleAxisd(
                  omega_a.norm() / 3.0,
                  omega_a / omega_a.norm())); // q1 = qa*exp(wa/3.0)
  }

  if (omega_b.norm() < 1e-6) {
    q2 = qb * Eigen::Quaterniond(1, 0, 0, 0);
  } else {
    q2 = qb * Eigen::Quaterniond(Eigen::AngleAxisd(
                  omega_b.norm() / 3.0,
                  -omega_b / omega_b.norm())); // q2 = qb*exp(wb/3.0)^-1
  }

  q3 = qb;

  // std::cout << "q0" << std::endl;
  // printQuat(q0);
  // std::cout << "q1" << std::endl;
  // printQuat(q1);
  // std::cout << "q2" << std::endl;
  // printQuat(q2);
  // std::cout << "q3" << std::endl;
  // printQuat(q3);

  // for world frame angular velocities, do: q_1*q_0.inverse(). for local frame
  // do: q_0.inverse()*q_1

  // Global Frame
  omega_1aa = q1 * q0.inverse();
  omega_2aa = q2 * q1.inverse();
  omega_3aa = q3 * q2.inverse();

  // Local Frame:
  // omega_1aa = q0.inverse()*q1;
  // omega_2aa = q1.inverse()*q2;
  // omega_3aa = q2.inverse()*q3;

  omega_1 = omega_1aa.axis() * omega_1aa.angle();
  omega_2 = omega_2aa.axis() * omega_2aa.angle();
  omega_3 = omega_3aa.axis() * omega_3aa.angle();

  // std::cout << "omega_1:" << omega_1.transpose() << std::endl;
  // std::cout << "omega_2:" << omega_2.transpose() << std::endl;
  // std::cout << "omega_3:" << omega_3.transpose() << std::endl;
}

void HermiteQuaternionCurve::computeBasis(const double &s_in) {
  s_ = util::Clamp(s_in, 0., 1.);
  b1 = 1 - std::pow((1 - s_), 3);
  b2 = 3 * std::pow(s_, 2) - 2 * std::pow((s_), 3);
  b3 = std::pow(s_, 3);

  bdot1 = 3 * std::pow((1 - s_), 2);
  bdot2 = 6 * s_ - 6 * std::pow((s_), 2);
  bdot3 = 3 * std::pow((s_), 2);

  bddot1 = -6 * (1 - s_);
  bddot2 = 6 - 12 * s_;
  bddot3 = 6 * s_;
}

void HermiteQuaternionCurve::Evaluate(const double &s_in,
                                      Eigen::Quaterniond &quat_out) {
  s_ = util::Clamp(s_in, 0., 1.);
  computeBasis(s_);

  qtmp1 = Eigen::AngleAxisd(omega_1aa.angle() * b1, omega_1aa.axis());
  qtmp2 = Eigen::AngleAxisd(omega_2aa.angle() * b2, omega_2aa.axis());
  qtmp3 = Eigen::AngleAxisd(omega_3aa.angle() * b3, omega_3aa.axis());

  // quat_out = q0*qtmp1*qtmp2*qtmp3; // local frame
  quat_out = qtmp3 * qtmp2 * qtmp1 * q0; // global frame
}

void HermiteQuaternionCurve::GetAngularVelocity(const double &s_in,
                                                Eigen::Vector3d &ang_vel_out) {
  // world frame: w(t) = qdot(t)*q^-1(t)
  // local frame: w(t) = q^-1(t)*qdot(t)
  // s_ = util::Clamp(s_in, 0., 1.);
  // computeBasis(s_);

  // qtmp1 = Eigen::AngleAxisd(omega_1aa.angle()*b1, omega_1aa.axis());
  // qtmp2 = Eigen::AngleAxisd(omega_2aa.angle()*b2, omega_2aa.axis());
  // qtmp3 = Eigen::AngleAxisd(omega_3aa.angle()*b3, omega_3aa.axis());

  // Eigen::Quaterniond q1dot; q1dot.vec() = omega_1*bdot1; q1dot *= qtmp1;
  // Eigen::Quaterniond q2dot; q2dot.vec() = omega_2*bdot2; q2dot *= qtmp2;
  // Eigen::Quaterniond q3dot; q3dot.vec() = omega_3*bdot3; q3dot *= qtmp3;

  // // Computing global frame angular velocity
  // Eigen::Quaterniond quat_dot;
  // Eigen::Quaterniond quat_out = qtmp3*qtmp2*qtmp1*q0; // global frame

  // quat_dot.vec() = (q3dot*(qtmp2*qtmp1*q0)).vec() +
  // (q2dot*(qtmp3*qtmp1*q0)).vec() + (q1dot*(qtmp3*qtmp2*q0)).vec();
  // quat_dot.w() = (q3dot*(qtmp2*qtmp1*q0)).w() + (q2dot*(qtmp3*qtmp1*q0)).w()
  // + (q1dot*(qtmp3*qtmp2*q0)).w();

  // ang_vel_out = (quat_dot*quat_out.conjugate()).vec(); //omega_1*b1 + omega_2
  // * b2 + omega_3 * b3;

  s_ = util::Clamp(s_in, 0., 1.);
  computeBasis(s_);
  ang_vel_out = omega_1 * bdot1 + omega_2 * bdot2 + omega_3 * bdot3;
}

// For world frame
void HermiteQuaternionCurve::GetAngularAcceleration(
    const double &s_in, Eigen::Vector3d &ang_acc_out) {
  // world frame: a(t) = (qddot(t)*q^-1(t) - (qdot(t)*q^-1(t))^2)
  // local frame: a(t) = (q^-1(t)*qddot(t) - (q^-1(t)*qdot(t))^2)

  // Numerically differentiate since we know the angular velocity
  // bool forward_diff = true;
  // if (s_ > 0.5){
  //   forward_diff = false;
  // }

  // double ds = 1e-6;
  // double s_query = s_in;
  // Eigen::Vector3d ang_vel_1, ang_vel_2;
  // if (forward_diff){
  //   s_query = s_in + ds;
  //   getAngularVelocity(s_query, ang_vel_2);
  //   s_query = s_in;
  //   getAngularVelocity(s_query, ang_vel_1);
  // }else {
  //   s_query = s_in;
  //   getAngularVelocity(s_query, ang_vel_2);
  //   s_query = s_in - ds;
  //   getAngularVelocity(s_query, ang_vel_1);
  // }
  // ang_acc_out = (ang_vel_2 - ang_vel_1)/ds;

  s_ = util::Clamp(s_in, 0., 1.);
  computeBasis(s_);
  ang_acc_out = omega_1 * bddot1 + omega_2 * bddot2 + omega_3 * bddot3;
}

MinJerkCurve::MinJerkCurve() { Initialization(); }

MinJerkCurve::MinJerkCurve(const Eigen::Vector3d &init,
                           const Eigen::Vector3d &end, const double &time_start,
                           const double &time_end) {
  Initialization();
  SetParams(init, end, time_start, time_end);
}

// Destructor
MinJerkCurve::~MinJerkCurve() {}

void MinJerkCurve::Initialization() {
  // Initialize to the corrrect sizes
  C_mat = Eigen::MatrixXd::Zero(6, 6);
  C_mat_inv = Eigen::MatrixXd::Zero(6, 6);
  a_coeffs = Eigen::VectorXd::Zero(6);
  bound_cond = Eigen::VectorXd::Zero(6);

  init_cond = Eigen::VectorXd::Zero(3);
  end_cond = Eigen::VectorXd::Zero(3);
  to = 0.0;
  tf = 1.0;
}

void MinJerkCurve::SetParams(const Eigen::Vector3d &init,
                             const Eigen::Vector3d &end,
                             const double &time_start, const double &time_end) {
  // Set the Parameters
  init_cond = init;
  end_cond = end;
  bound_cond.head(3) = init_cond;
  bound_cond.tail(3) = end_cond;
  to = time_start;
  tf = time_end;

  // Construct C matrix
  C_mat(0, 0) = 1.0;
  C_mat(0, 1) = to;
  C_mat(0, 2) = std::pow(to, 2);
  C_mat(0, 3) = std::pow(to, 3);
  C_mat(0, 4) = std::pow(to, 4);
  C_mat(0, 5) = std::pow(to, 5);
  C_mat(1, 1) = 1.0;
  C_mat(1, 2) = 2.0 * to;
  C_mat(1, 3) = 3.0 * std::pow(to, 2);
  C_mat(1, 4) = 4.0 * std::pow(to, 3);
  C_mat(1, 5) = 5.0 * std::pow(to, 4);
  C_mat(2, 2) = 2.0;
  C_mat(2, 3) = 6.0 * to;
  C_mat(2, 4) = 12.0 * std::pow(to, 2);
  C_mat(2, 5) = 20.0 * std::pow(to, 3);
  C_mat(3, 0) = 1.0;
  C_mat(3, 1) = tf;
  C_mat(3, 2) = std::pow(tf, 2);
  C_mat(3, 3) = std::pow(tf, 3);
  C_mat(3, 4) = std::pow(tf, 4);
  C_mat(3, 5) = std::pow(tf, 5);
  C_mat(4, 1) = 1.0;
  C_mat(4, 2) = 2.0 * tf;
  C_mat(4, 3) = 3.0 * std::pow(tf, 2);
  C_mat(4, 4) = 4.0 * std::pow(tf, 3);
  C_mat(4, 5) = 5.0 * std::pow(tf, 4);
  C_mat(5, 2) = 2.0;
  C_mat(5, 3) = 6.0 * tf;
  C_mat(5, 4) = 12.0 * std::pow(tf, 2);
  C_mat(5, 5) = 20.0 * std::pow(tf, 3);

  // Solve for the coefficients
  C_mat_inv = C_mat.inverse();
  a_coeffs = C_mat_inv * bound_cond;
}

void MinJerkCurve::GetPos(const double &time, double &pos) {
  double t;
  if (time <= to) {
    t = to;
  } else if (time >= tf) {
    t = tf;
  } else {
    t = time;
  }
  pos = a_coeffs[0] + a_coeffs[1] * t + a_coeffs[2] * std::pow(t, 2) +
        a_coeffs[3] * std::pow(t, 3) + a_coeffs[4] * std::pow(t, 4) +
        a_coeffs[5] * std::pow(t, 5);
}
void MinJerkCurve::GetVel(const double &time, double &vel) {
  double t;
  if (time <= to) {
    t = to;
  } else if (time >= tf) {
    t = tf;
  } else {
    t = time;
  }
  vel = a_coeffs[1] + 2.0 * a_coeffs[2] * t +
        3.0 * a_coeffs[3] * std::pow(t, 2) +
        4.0 * a_coeffs[4] * std::pow(t, 3) + 5.0 * a_coeffs[5] * std::pow(t, 4);
}
void MinJerkCurve::GetAcc(const double &time, double &acc) {
  double t;
  if (time <= to) {
    t = to;
  } else if (time >= tf) {
    t = tf;
  } else {
    t = time;
  }
  acc = 2.0 * a_coeffs[2] + 6.0 * a_coeffs[3] * t +
        12.0 * a_coeffs[4] * std::pow(t, 2) +
        20.0 * a_coeffs[5] * std::pow(t, 3);
}

MinJerkCurveVec::MinJerkCurveVec() {}

MinJerkCurveVec::MinJerkCurveVec(const Eigen::VectorXd &start_pos,
                                 const Eigen::VectorXd &start_vel,
                                 const Eigen::VectorXd &start_acc,
                                 const Eigen::VectorXd &end_pos,
                                 const Eigen::VectorXd &end_vel,
                                 const Eigen::VectorXd &end_acc,
                                 double duration)
    : p1_(start_pos), v1_(start_vel), a1_(start_acc), p2_(end_pos),
      v2_(end_vel), a2_(end_acc), Ts_(duration) {

  // Create N minjerk curves_ with the specified boundary conditions
  for (int i = 0; i < start_pos.size(); i++) {
    curves_.push_back(MinJerkCurve(Eigen::Vector3d(p1_[i], v1_[i], a1_[i]),
                                   Eigen::Vector3d(p2_[i], v2_[i], a2_[i]), 0.0,
                                   Ts_));
  }
  output_ = Eigen::VectorXd::Zero(start_pos.size());
}

// Destructor
MinJerkCurveVec::~MinJerkCurveVec() {}

// Evaluation functions
Eigen::VectorXd MinJerkCurveVec::Evaluate(const double &t_in) {
  double val;
  for (int i = 0; i < p1_.size(); i++) {
    curves_[i].GetPos(t_in, val);
    output_[i] = val;
  }
  return output_;
}

Eigen::VectorXd MinJerkCurveVec::EvaluateFirstDerivative(const double &t_in) {
  double val;
  for (int i = 0; i < v1_.size(); i++) {
    curves_[i].GetVel(t_in, val);
    output_[i] = val;
  }
  return output_;
}

Eigen::VectorXd MinJerkCurveVec::EvaluateSecondDerivative(const double &t_in) {
  double val;
  for (int i = 0; i < a1_.size(); i++) {
    curves_[i].GetAcc(t_in, val);
    output_[i] = val;
  }
  return output_;
}
