#include <dart/dart.hpp>

#include <Configuration.h>
#include <PnC/A1PnC/A1Definition.hpp>
#include <PnC/A1PnC/A1StateEstimator/BasicAccumulation.hpp>
#include <Utils/IO/DataManager.hpp>
#include <Utils/IO/IOUtilities.hpp>

BasicAccumulation::BasicAccumulation() : filtered_acc_(3) {
  myUtils::pretty_constructor(2, "Basic Accumulation");
  global_ori_quat_.w() = 1.;
  global_ori_quat_.x() = 0.;
  global_ori_quat_.y() = 0.;
  global_ori_quat_.z() = 0.;
  global_ori_ypr_.setZero(0);

  cutoff_freq_ = 2.0 * M_PI * 1.0;  // 1Hz // (2*pi*frequency) rads/s
  for (int i(0); i < 3; ++i) {
    filtered_acc_[i] =
        new digital_lp_filter(cutoff_freq_, A1Aux::servo_rate);
  }
  global_ang_vel_.setZero();
}
void BasicAccumulation::estimatorInitialization(
    const std::vector<double>& acc, const std::vector<double>& ang_vel) {
  for (int i(0); i < 3; ++i) {
    global_ang_vel_[i] = ang_vel[i];
    filtered_acc_[i]->input(acc[i]);
  }
  _InitIMUOrientationEstimateFromGravity();
}

void BasicAccumulation::setSensorData(const std::vector<double>& acc,
                                      const std::vector<double>& ang_vel) {
  // Convert body omega into a delta quaternion ------------------------------
  Eigen::Vector3d body_omega;
  body_omega.setZero();
  for (size_t i = 0; i < 3; i++) {
    body_omega[i] = ang_vel[i];
  }
  Eigen::Quaternion<double> delta_quat_body =
      dart::math::expToQuat(body_omega * A1Aux::servo_rate);

  Eigen::MatrixXd R_global_to_torso =
      global_ori_quat_.normalized().toRotationMatrix();
  global_ang_vel_ = R_global_to_torso * body_omega;

  // Perform orientation update via integration
  global_ori_quat_ = global_ori_quat_ * delta_quat_body;
  global_ori_ypr_ = dart::math::matrixToEulerZYX(
      global_ori_quat_.normalized().toRotationMatrix());
  global_ori_ypr_dot_ = _so3_to_euler_zyx_dot(global_ori_ypr_, global_ang_vel_);
}

void BasicAccumulation::_InitIMUOrientationEstimateFromGravity() {
  Eigen::Vector3d g_B;
  g_B.setZero();
  for (int i(0); i < 3; ++i) {
    // We expect a negative number if gravity is pointing opposite of
    // the IMU direction
    g_B[i] = filtered_acc_[i]->output();
  }
  // Test Vector  ////////////////////////
  // g_B[0] = 0.1; g_B[1] = 0.4; g_B[2] = 0.5;
  // g_B[0] = 0.0; g_B[1] = 0.0; g_B[2] = 1.0;
  // qd rot_quat;
  // dynacore::convert(1.0, -0.5, 0.1, rot_quat);
  // Eigen::Matrix3d rot_mt(rot_quat);
  // g_B = rot_mt * g_B;
  //////////////////////////////////////////////
  g_B.normalize();

  double theta_pitch = atan2(g_B[0], g_B[2]);

  Eigen::Vector3d g_B_xz = g_B;
  g_B_xz[1] = 0.;
  g_B_xz.normalize();

  double inner = g_B_xz.transpose() * g_B;
  double theta_roll = acos(inner);
  if (g_B[1] > 0) theta_roll *= (-1.);

  // Eigen::Quaternion<double> quat_pitch(dart::math::eulerYZXToMatrix(
  // Eigen::Vector3d(0., theta_pitch, 0.)));
  Eigen::Quaternion<double> quat_pitch(
      dart::math::eulerYZXToMatrix(Eigen::Vector3d(theta_pitch, 0., 0.)));
  Eigen::Quaternion<double> quat_roll(
      dart::math::eulerYZXToMatrix(Eigen::Vector3d(0., 0., theta_roll)));
  Eigen::Quaternion<double> local2Glob = quat_pitch * quat_roll;
  global_ori_quat_ = local2Glob.inverse();
  global_ori_ypr_ = dart::math::matrixToEulerZYX(
      global_ori_quat_.normalized().toRotationMatrix());
  global_ori_ypr_dot_ = _so3_to_euler_zyx_dot(global_ori_ypr_, global_ang_vel_);

  ///////////////////////////////////////////////////////////////////
  // Eigen::Matrix3d Global_ori(global_ori_quat_);
  // v3 check_vec = Global_ori * g_B;
  // dynacore::pretty_print(check_vec, std::cout, "check vec");
  // dynacore::pretty_print(g_B, std::cout, "g_B");
  // printf("pitch, roll: %f, %f\n", theta_pitch, theta_roll);
}

Eigen::Vector3d BasicAccumulation::_so3_to_euler_zyx_dot(
    const Eigen::Vector3d& _global_ori_ypr,
    const Eigen::Vector3d& _global_ang_vel) {
  Eigen::MatrixXd so3_to_euler_zyx_dot_map(3, 3);
  double x(_global_ori_ypr[2]);
  double y(_global_ori_ypr[1]);
  double z(_global_ori_ypr[0]);

  if (y == M_PI / 2.0) {
    std::cout << "Singular at mapping from euler zyx to so3" << std::endl;
    exit(0);
  }

  so3_to_euler_zyx_dot_map << cos(z) * sin(y) / cos(y),
      sin(y) * sin(z) / cos(y), 1, -sin(z), cos(z), 0, cos(z) / cos(y),
      sin(z) / cos(y), 0;
  return so3_to_euler_zyx_dot_map * _global_ang_vel;
}
