#pragma once

#include <algorithm>
#include <fstream>
#include <iostream>
#include <list>
#include <stdio.h>

#include <Eigen/Dense>

#include "configuration.hpp"
#include "utils/util.hpp"
#include "hermite_curve_vec.hpp"

namespace util {
double SmoothPos(double ini, double end, double moving_duration,
                 double curr_time);
double SmoothVel(double ini, double end, double moving_duration,
                 double curr_time);
double SmoothAcc(double ini, double end, double moving_duration,
                 double curr_time);
void SinusoidTrajectory(double initTime_, const Eigen::VectorXd &midPoint_,
                        const Eigen::VectorXd &amp_,
                        const Eigen::VectorXd &freq_, double evalTime_,
                        Eigen::VectorXd &p_, Eigen::VectorXd &v_,
                        Eigen::VectorXd &a_, double smoothing_dur = 1.0);
double Smooth(double ini, double fin, double rat);
} // namespace util

// class HermiteCurve {
// public:
// HermiteCurve();
// HermiteCurve(const double &start_pos, const double &start_vel,
// const double &end_pos, const double &end_vel);
//~HermiteCurve();
// void Initialize(const double &start_pos, const double &start_vel,
// const double &end_pos, const double &end_vel);
// double Evaluate(const double &s_in);
// double EvaluateFirstDerivative(const double &s_in);
// double EvaluateSecondDerivative(const double &s_in);

// private:
// double p1_;
// double v1_;
// double p2_;
// double v2_;

// double s_;
//};

// class HermiteCurveVec {
// public:
// HermiteCurveVec();
// HermiteCurveVec(const Eigen::VectorXd &start_pos,
// const Eigen::VectorXd &start_vel,
// const Eigen::VectorXd &end_pos,
// const Eigen::VectorXd &end_vel);

// void Initialize(const Eigen::VectorXd &start_pos,
// const Eigen::VectorXd &start_vel,
// const Eigen::VectorXd &end_pos,
// const Eigen::VectorXd &end_vel);

//~HermiteCurveVec();
// Eigen::VectorXd Evaluate(const double &s_in);
// Eigen::VectorXd EvaluateFirstDerivative(const double &s_in);
// Eigen::VectorXd EvaluateSecondDerivative(const double &s_in);

// private:
// Eigen::VectorXd p1_;
// Eigen::VectorXd v1_;
// Eigen::VectorXd p2_;
// Eigen::VectorXd v2_;

// std::vector<HermiteCurve> curves_;
// Eigen::VectorXd output_;
//};

// class HermiteQuaternionCurve {
// public:
// HermiteQuaternionCurve();
// HermiteQuaternionCurve(const Eigen::Quaterniond &quat_start,
// const Eigen::Vector3d &angular_velocity_start,
// const Eigen::Quaterniond &quat_end,
// const Eigen::Vector3d &angular_velocity_end);
//~HermiteQuaternionCurve();

// void Initialize(const Eigen::Quaterniond &quat_start,
// const Eigen::Vector3d &angular_velocity_start,
// const Eigen::Quaterniond &quat_end,
// const Eigen::Vector3d &angular_velocity_end);

// All values are expressed in "world frame"
// void Evaluate(const double &s_in, Eigen::Quaterniond &quat_out);
// void GetAngularVelocity(const double &s_in, Eigen::Vector3d &ang_vel_out);
// void GetAngularAcceleration(const double &s_in, Eigen::Vector3d
// &ang_acc_out);

// private:
// Eigen::Quaterniond qa;   // Starting quaternion
// Eigen::Vector3d omega_a; // Starting Angular Velocity
// Eigen::Quaterniond qb;   // Ending quaternion
// Eigen::Vector3d omega_b; // Ending Angular velocity

// Eigen::AngleAxisd omega_a_aa; // axis angle representation of omega_a
// Eigen::AngleAxisd omega_b_aa; // axis angle representation of omega_b

// void initialize_data_structures();

// void computeBasis(const double &s_in); // computes the basis functions
// void computeOmegas();

// Eigen::Quaterniond q0; // quat0
// Eigen::Quaterniond q1; // quat1
// Eigen::Quaterniond q2; // quat1
// Eigen::Quaterniond q3; // quat1

// double b1; // basis 1
// double b2; // basis 2
// double b3; // basis 3

// double bdot1; // 1st derivative of basis 1
// double bdot2; // 1st derivative of basis 2
// double bdot3; // 1st derivative of basis 3

// double bddot1; // 2nd derivative of basis 1
// double bddot2; // 2nd derivative of basis 2
// double bddot3; // 2nd derivative of basis 3

// Eigen::Vector3d omega_1;
// Eigen::Vector3d omega_2;
// Eigen::Vector3d omega_3;

// Eigen::AngleAxisd omega_1aa;
// Eigen::AngleAxisd omega_2aa;
// Eigen::AngleAxisd omega_3aa;

// Allocate memory for quaternion operations
// Eigen::Quaterniond qtmp1;
// Eigen::Quaterniond qtmp2;
// Eigen::Quaterniond qtmp3;

// progression variable
// double s_;
//};

class MinJerkCurve {
public:
  // Constructors
  MinJerkCurve();
  MinJerkCurve(const Eigen::Vector3d &init, const Eigen::Vector3d &end,
               const double &time_start, const double &time_end);

  void SetParams(const Eigen::Vector3d &init, const Eigen::Vector3d &end,
                 const double &time_start, const double &time_end);

  void GetPos(const double &time, double &pos);
  void GetVel(const double &time, double &vel);
  void GetAcc(const double &time, double &acc);

  // Destructor
  ~MinJerkCurve();

private:
  Eigen::MatrixXd C_mat;     // Matrix of Coefficients
  Eigen::MatrixXd C_mat_inv; // Inverse of Matrix of Coefficients
  Eigen::VectorXd
      a_coeffs; // mininum jerk coeffs. a = [a0, a1, a2, a3, a4, a5, a6];
  Eigen::VectorXd bound_cond; // boundary conditions x_b = [ x(to), xdot(to),
                              // xddot(to), x(tf), xdot(tf), xddot(tf)]

  Eigen::Vector3d init_cond; // initial pos, vel, acceleration
  Eigen::Vector3d end_cond;  // final pos, vel, acceleration
  double to;                 // Starting time
  double tf;                 // Ending time

  void Initialization();

  // Compute the coefficients
  void compute_coeffs();
};

class MinJerkCurveVec {
public:
  MinJerkCurveVec();
  MinJerkCurveVec(const Eigen::VectorXd &start_pos,
                  const Eigen::VectorXd &start_vel,
                  const Eigen::VectorXd &start_acc,
                  const Eigen::VectorXd &end_pos,
                  const Eigen::VectorXd &end_vel,
                  const Eigen::VectorXd &end_acc, double duration);
  ~MinJerkCurveVec();
  Eigen::VectorXd Evaluate(const double &t_in);
  Eigen::VectorXd EvaluateFirstDerivative(const double &t_in);
  Eigen::VectorXd EvaluateSecondDerivative(const double &t_in);

private:
  double Ts_;

  Eigen::VectorXd p1_;
  Eigen::VectorXd v1_;
  Eigen::VectorXd a1_;

  Eigen::VectorXd p2_;
  Eigen::VectorXd v2_;
  Eigen::VectorXd a2_;

  std::vector<MinJerkCurve> curves_;
  Eigen::VectorXd output_;
};

//class HermiteCurve {
//public:
//  HermiteCurve();
//  HermiteCurve(const double &start_pos, const double &start_vel,
//               const double &end_pos, const double &end_vel,
//               const double &duration);
//  ~HermiteCurve();
//  double evaluate(const double &t_in);
//  double evaluateFirstDerivative(const double &t_in);
//  double evaluateSecondDerivative(const double &t_in);
//
//private:
//  double p1;
//  double v1;
//  double p2;
//  double v2;
//
//  double t_dur;
//
//  double s_;
//
//  // by default clamps within 0 and 1.
//  double clamp(const double &t_in, double lo = 0.0, double hi = 1.0);
//};
//
//class HermiteCurveVec {
//public:
//  HermiteCurveVec();
//  HermiteCurveVec(const Eigen::VectorXd &start_pos,
//                  const Eigen::VectorXd &start_vel,
//                  const Eigen::VectorXd &end_pos,
//                  const Eigen::VectorXd &end_vel, const double &duration);
//  ~HermiteCurveVec();
//
//  void initialize(const Eigen::VectorXd &start_pos,
//                  const Eigen::VectorXd &start_vel,
//                  const Eigen::VectorXd &end_pos,
//                  const Eigen::VectorXd &end_vel, const double &duration);
//  Eigen::VectorXd evaluate(const double &t_in);
//  Eigen::VectorXd evaluateFirstDerivative(const double &t_in);
//  Eigen::VectorXd evaluateSecondDerivative(const double &t_in);
//
//private:
//  Eigen::VectorXd p1;
//  Eigen::VectorXd v1;
//  Eigen::VectorXd p2;
//  Eigen::VectorXd v2;
//
//  double t_dur;
//
//  std::vector<HermiteCurve> curves;
//  Eigen::VectorXd output;
//};
//
//class HermiteQuinticCurve {
//public:
//    HermiteQuinticCurve();
//    HermiteQuinticCurve(const double &start_pos, const double &start_vel,
//                 const double &start_accel, const double &end_pos,
//                 const double &end_vel, const double &end_accel,
//                 const double &duration);
//    ~HermiteQuinticCurve();
//    double evaluate(const double &t_in);
//    double evaluateFirstDerivative(const double &t_in);
//    double evaluateSecondDerivative(const double &t_in);
//
//private:
//    double p1;
//    double v1;
//    double a1;
//    double p2;
//    double v2;
//    double a2;
//
//    double t_dur;
//
//    double s_;
//
//    // by default clamps within 0 and 1.
//    double clamp(const double &t_in, double lo = 0.0, double hi = 1.0);
//};
//
//class HermiteQuinticCurveVec {
//public:
//    HermiteQuinticCurveVec();
//    HermiteQuinticCurveVec(const Eigen::VectorXd &start_pos,
//                    const Eigen::VectorXd &start_vel,
//                    const Eigen::VectorXd &start_accel,
//                    const Eigen::VectorXd &end_pos,
//                    const Eigen::VectorXd &end_vel,
//                    const Eigen::VectorXd &end_accel, const double &duration);
//    ~HermiteQuinticCurveVec();
//
//    void initialize(const Eigen::VectorXd &start_pos,
//                    const Eigen::VectorXd &start_vel,
//                    const Eigen::VectorXd &start_accel,
//                    const Eigen::VectorXd &end_pos,
//                    const Eigen::VectorXd &end_vel,
//                    const Eigen::VectorXd &end_accel, const double &duration);
//    Eigen::VectorXd evaluate(const double &t_in);
//    Eigen::VectorXd evaluateFirstDerivative(const double &t_in);
//    Eigen::VectorXd evaluateSecondDerivative(const double &t_in);
//
//private:
//    Eigen::VectorXd p1;
//    Eigen::VectorXd v1;
//    Eigen::VectorXd a1;
//    Eigen::VectorXd p2;
//    Eigen::VectorXd v2;
//    Eigen::VectorXd a2;
//
//    double t_dur;
//
//    std::vector<HermiteQuinticCurve> curves;
//    Eigen::VectorXd output;
//};

// Hermite Quaternion curve for global frame quaternion trajectory given
// boundary conditions also computes global frame angular velocity and angular
// acceleration for s \in [0,1]

class HermiteQuaternionCurve {
public:
  HermiteQuaternionCurve();
  HermiteQuaternionCurve(const Eigen::Quaterniond &quat_start,
                         const Eigen::Vector3d &angular_velocity_start,
                         const Eigen::Quaterniond &quat_end,
                         const Eigen::Vector3d &angular_velocity_end,
                         double duration);
  ~HermiteQuaternionCurve();

  void initialize(const Eigen::Quaterniond &quat_start,
                  const Eigen::Vector3d &angular_velocity_start,
                  const Eigen::Quaterniond &quat_end,
                  const Eigen::Vector3d &angular_velocity_end, double duration);

  // All values are expressed in "world frame"
  void evaluate(const double &t_in, Eigen::Quaterniond &quat_out);
  void getAngularVelocity(const double &t_in, Eigen::Vector3d &ang_vel_out);
  void getAngularAcceleration(const double &t_in, Eigen::Vector3d &ang_acc_out);

private:
  double t_dur; // time duration

  Eigen::Quaterniond qa;   // Starting quaternion
  Eigen::Vector3d omega_a; // Starting Angular Velocity
  Eigen::Quaterniond qb;   // Ending quaternion
  Eigen::Vector3d omega_b; // Ending Angular velocity

  void initialize_data_structures();
  HermiteCurveVec theta_ab; // so3
  Eigen::Quaterniond delq;

  ///////////////////////////////////////

  Eigen::AngleAxisd omega_a_aa; // axis angle representation of omega_a
  Eigen::AngleAxisd omega_b_aa; // axis angle representation of omega_b

  void computeBasis(const double &t_in); // computes the basis functions
  void computeOmegas();

  Eigen::Quaterniond q0; // quat0
  Eigen::Quaterniond q1; // quat1
  Eigen::Quaterniond q2; // quat1
  Eigen::Quaterniond q3; // quat1

  double b1; // basis 1
  double b2; // basis 2
  double b3; // basis 3

  double bdot1; // 1st derivative of basis 1
  double bdot2; // 1st derivative of basis 2
  double bdot3; // 1st derivative of basis 3

  double bddot1; // 2nd derivative of basis 1
  double bddot2; // 2nd derivative of basis 2
  double bddot3; // 2nd derivative of basis 3

  Eigen::Vector3d omega_1;
  Eigen::Vector3d omega_2;
  Eigen::Vector3d omega_3;

  Eigen::AngleAxisd omega_1aa;
  Eigen::AngleAxisd omega_2aa;
  Eigen::AngleAxisd omega_3aa;

  // Allocate memory for quaternion operations
  Eigen::Quaterniond qtmp1;
  Eigen::Quaterniond qtmp2;
  Eigen::Quaterniond qtmp3;

  void printQuat(const Eigen::Quaterniond &quat);
};
