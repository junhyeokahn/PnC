#include <Utils/Math/MathUtilities.hpp>
#include <cassert>
#include <cmath>

namespace myUtils {
double smoothing(double ini, double fin, double rat) {
  double ret(0.);
  if (rat < 0) {
    return ini;
  } else if (rat > 1) {
    return fin;
  } else {
    return ini + (fin - ini) * rat;
  }
}

Eigen::MatrixXd hStack(const Eigen::MatrixXd &a, const Eigen::MatrixXd &b) {
  assert(a.rows() == b.rows());
  Eigen::MatrixXd ab = Eigen::MatrixXd::Zero(a.rows(), a.cols() + b.cols());
  ab << a, b;
  return ab;
}

Eigen::MatrixXd vStack(const Eigen::MatrixXd &a, const Eigen::MatrixXd &b) {
  assert(a.cols() == b.cols());
  Eigen::MatrixXd ab = Eigen::MatrixXd::Zero(a.rows() + b.rows(), a.cols());
  ab << a, b;
  return ab;
}

Eigen::MatrixXd block_diag(const Eigen::MatrixXd &_a,
                           const Eigen::MatrixXd &_b) {
  Eigen::MatrixXd ret =
      Eigen::MatrixXd::Zero(_a.rows() + _b.rows(), _a.cols() + _b.cols());
  ret.block(0, 0, _a.rows(), _a.cols()) = _a;
  ret.block(_a.rows(), _a.cols(), _b.rows(), _b.cols()) = _b;
  return ret;
}

Eigen::MatrixXd deleteRow(const Eigen::MatrixXd &a_, int row_) {
  Eigen::MatrixXd ret = Eigen::MatrixXd::Zero(a_.rows() - 1, a_.cols());
  ret.block(0, 0, row_, a_.cols()) = a_.block(0, 0, row_, a_.cols());
  ret.block(row_, 0, ret.rows() - row_, a_.cols()) =
      a_.block(row_ + 1, 0, ret.rows() - row_, a_.cols());
  return ret;
}

double smooth_changing(double ini, double end, double moving_duration,
                       double curr_time) {
  double ret;
  ret = ini + (end - ini) * 0.5 * (1 - cos(curr_time / moving_duration * M_PI));
  if (curr_time > moving_duration) {
    ret = end;
  }
  return ret;
}

double smooth_changing_vel(double ini, double end, double moving_duration,
                           double curr_time) {
  double ret;
  ret = (end - ini) * 0.5 * (M_PI / moving_duration) *
        sin(curr_time / moving_duration * M_PI);
  if (curr_time > moving_duration) {
    ret = 0.0;
  }
  return ret;
}
double smooth_changing_acc(double ini, double end, double moving_duration,
                           double curr_time) {
  double ret;
  ret = (end - ini) * 0.5 * (M_PI / moving_duration) *
        (M_PI / moving_duration) * cos(curr_time / moving_duration * M_PI);
  if (curr_time > moving_duration) {
    ret = 0.0;
  }
  return ret;
}

void getSinusoidTrajectory(double initTime_, const Eigen::VectorXd &midPoint_,
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

double computeAlphaGivenBreakFrequency(double hz, double dt) {
  double omega = 2.0 * M_PI * hz;
  double alpha = (1.0 - (omega * dt / 2.0)) / (1.0 + (omega * dt / 2.0));
  alpha = CropValue(alpha, 0.0, 1.0);
  return alpha;
}

double bind_half_pi(double ang) {
  if (ang > M_PI / 2) {
    return ang - M_PI;
  }
  if (ang < -M_PI / 2) {
    return ang + M_PI;
  }
  return ang;
}

bool isInBoundingBox(const Eigen::VectorXd &val, const Eigen::VectorXd &lb,
                     const Eigen::VectorXd &ub) {
  int n = lb.size();
  bool ret(true);
  for (int i = 0; i < n; ++i) {
    if (lb[i] <= val[i] && val[i] <= ub[i]) {
    } else {
      myUtils::color_print(myColor::BoldMagneta, "Is not BoundingBox");
      std::cout << i << " th : lb = " << lb[i] << " val = " << val[i]
                << " ub = " << ub[i] << std::endl;
      ret = false;
    }
  }
  return ret;
}

Eigen::VectorXd eulerIntegration(const Eigen::VectorXd &x,
                                 const Eigen::VectorXd &xdot, double dt) {
  Eigen::VectorXd ret = x;
  ret += xdot * dt;
  return ret;
}

Eigen::VectorXd doubleIntegration(const Eigen::VectorXd &q,
                                  const Eigen::VectorXd &alpha,
                                  const Eigen::VectorXd &alphad, double dt) {
  Eigen::VectorXd ret = q;
  ret += alpha * dt + alphad * dt * dt * 0.5;
  return ret;
}

double CropValue(double value, double min, double max, std::string source) {
  assert(min < max);
  if (value > max) {
    printf("%s: %f is cropped to %f.\n", source.c_str(), value, max);
    value = max;
  }
  if (value < min) {
    printf("%s: %f is cropped to %f.\n", source.c_str(), value, min);
    value = min;
  }
  return value;
}

double CropValue(double value, double min, double max) {
  assert(min < max);
  if (value > max) {
    value = max;
  }
  if (value < min) {
    value = min;
  }
  return value;
}

Eigen::VectorXd CropVector(Eigen::VectorXd value, Eigen::VectorXd min,
                           Eigen::VectorXd max, std::string source) {
  assert(value.size() = min.size());
  assert(value.size() = max.size());
  int n_data = value.size();

  for (int i = 0; i < n_data; ++i) {
    if (value[i] > max[i]) {
      // printf("%s(%d): %f is cropped to %f\n", source.c_str(), i,
      // value[i], max[i]);
      value[i] = max[i];
    }
    if (value[i] < min[i]) {
      // printf("%s(%d): %f is cropped to %f\n", source.c_str(), i,
      // value[i], min[i]);
      value[i] = min[i];
    }
  }
  return value;
}

Eigen::MatrixXd CropMatrix(Eigen::MatrixXd value, Eigen::MatrixXd min,
                           Eigen::MatrixXd max, std::string source) {
  assert((value.cols() = min.cols()) && (value.cols() = max.cols()));
  assert((value.rows() = min.rows()) && (value.cols() = max.cols()));

  int n_row = value.rows();
  int n_cols = value.cols();

  for (int row_idx = 0; row_idx < n_row; ++row_idx) {
    for (int col_idx = 0; col_idx < n_cols; ++col_idx) {
      if (value(row_idx, col_idx) < min(row_idx, col_idx)) {
        // printf("%s(%d, %d): %f is cropped to %f\n", source.c_str(),
        // row_idx, col_idx, value(row_idx, col_idx), min(row_idx,
        // col_idx));
        value(row_idx, col_idx) = min(row_idx, col_idx);
      }
      if (value(row_idx, col_idx) > max(row_idx, col_idx)) {
        // printf("%s(%d, %d): %f is cropped to %f\n", source.c_str(),
        // row_idx, col_idx, value(row_idx, col_idx), max(row_idx,
        // col_idx));
        value(row_idx, col_idx) = max(row_idx, col_idx);
      }
    }
  }
  return value;
}

Eigen::MatrixXd GetRelativeMatrix(const Eigen::MatrixXd value,
                                  const Eigen::MatrixXd min,
                                  const Eigen::MatrixXd max) {
  assert((value.cols() = min.cols()) && (value.cols() = max.cols()));
  assert((value.rows() = min.rows()) && (value.cols() = max.cols()));

  Eigen::MatrixXd ret = value;
  for (int col_idx = 0; col_idx < value.cols(); ++col_idx) {
    for (int row_idx = 0; row_idx < value.rows(); ++row_idx) {
      double width = max(row_idx, col_idx) - min(row_idx, col_idx);
      double mid = (max(row_idx, col_idx) + min(row_idx, col_idx)) / 2.0;
      ret(row_idx, col_idx) = 2.0 * (value(row_idx, col_idx) - mid) / width;
    }
  }
  return ret;
}

Eigen::VectorXd GetRelativeVector(const Eigen::VectorXd value,
                                  const Eigen::VectorXd min,
                                  const Eigen::VectorXd max) {
  assert((value.size() = min.size()) && (value.size() = max.size()));
  Eigen::VectorXd ret = value;

  for (int idx = 0; idx < value.size(); ++idx) {
    double width = max(idx) - min(idx);
    double mid = (max(idx) + min(idx)) / 2.0;
    ret(idx) = 2.0 * (value(idx) - mid) / width;
  }
  return ret;
}

// From Modern Robotics
// Lynch, Kevin M., and Frank C. Park. Modern Robotics. Cambridge University
// Press, 2017.
// CPP Implementation: https://github.com/Le0nX/ModernRoboticsCpp
/* Function: Returns the skew symmetric matrix representation of an angular
 * velocity vector
 * Input: Eigen::Vector3d 3x1 angular velocity vector
 * Returns: Eigen::MatrixXd 3x3 skew symmetric matrix
 */
Eigen::Matrix3d VecToso3(const Eigen::Vector3d &omg) {
  Eigen::Matrix3d m_ret;
  m_ret << 0, -omg(2), omg(1), omg(2), 0, -omg(0), -omg(1), omg(0), 0;
  return m_ret;
}

// From Modern Robotics
// Lynch, Kevin M., and Frank C. Park. Modern Robotics. Cambridge University
// Press, 2017.
// CPP Implementation:  https://github.com/Le0nX/ModernRoboticsCpp
/* Function: Provides the adjoint representation of a transformation matrix
             Used to change the frame of reference for spatial velocity vectors
 * Inputs: Eigen::MatrixXd 3x3 Rotation matrix, Eigen::Vector3d, 3x1 translation
 vector
 * Returns: Eigen::MatrixXd 6x6 Adjoint matrix for transforming twists
 representation to a different frame
*/
Eigen::MatrixXd Adjoint(const Eigen::MatrixXd &R, const Eigen::Vector3d &p) {
  Eigen::MatrixXd ad_ret = Eigen::MatrixXd::Zero(6, 6);
  Eigen::MatrixXd zeroes = Eigen::MatrixXd::Zero(3, 3);
  ad_ret << R, zeroes, VecToso3(p) * R, R;
  return ad_ret;
}

Eigen::Vector3d quat_to_exp(const Eigen::Quaternion<double> &quat) {
  Eigen::Vector3d img_vec(quat.x(), quat.y(), quat.z());
  double w(quat.w());
  double theta(2.0 * std::asin(std::sqrt(img_vec[0] * img_vec[0] +
                                         img_vec[1] * img_vec[1] +
                                         img_vec[2] * img_vec[2])));
  if (theta < 0.0001) {
    return Eigen::Vector3d::Zero();
  }
  Eigen::Vector3d ret = img_vec / std::sin(theta / 2.);
  return ret * theta;
}

Eigen::Quaternion<double> exp_to_quat(const Eigen::Vector3d &exp) {

  double theta = exp.norm();
  Eigen::Quaternion<double> ret;

  if (theta > 1.0e-4) {
    ret.w() = cos(theta / 2.0);
    ret.x() = sin(theta / 2.0) * exp[0] / theta;
    ret.y() = sin(theta / 2.0) * exp[1] / theta;
    ret.z() = sin(theta / 2.0) * exp[2] / theta;
  } else {
    ret.w() = 1.;
    ret.x() = 0.5 * exp[0];
    ret.y() = 0.5 * exp[1];
    ret.z() = 0.5 * exp[2];
  }

  return ret;
}

double QuatToYaw(const Eigen::Quaternion<double> q) {
  // to match equation from:
  // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  const double &q0 = q.w();
  const double &q1 = q.x();
  const double &q2 = q.y();
  const double &q3 = q.z();

  return std::atan2(2. * (q0 * q3 + q1 * q2), 1. - 2. * (q2 * q2 + q3 * q3));
}

// Euler ZYX
//     Represents either:
//     extrinsic XYZ rotations: Fixed-frame roll, then fixed-frame pitch, then
//     fixed-frame yaw.
//     or intrinsic ZYX rotations: Body-frame yaw, body-frame pitch, then
//     body-frame roll
//
//     The equation is similar, but the values for fixed and body frame
//     rotations are different.
// World Orientation is R = Rz*Ry*Rx
Eigen::Quaterniond EulerZYXtoQuat(const double roll, const double pitch,
                                  const double yaw) {
  Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

  Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
  return q.normalized();
}

Eigen::Vector3d QuatToEulerZYX(const Eigen::Quaterniond &quat_in) {
  // to match equation from:
  // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

  // roll (x-axis rotation)
  double sinr_cosp =
      2 * (quat_in.w() * quat_in.x() + quat_in.y() * quat_in.z());
  double cosr_cosp =
      1 - 2 * (quat_in.x() * quat_in.x() + quat_in.y() * quat_in.y());
  double roll = std::atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = 2 * (quat_in.w() * quat_in.y() - quat_in.z() * quat_in.x());
  double pitch;
  if (std::abs(sinp) >= 1)
    pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
    pitch = std::asin(sinp);

  // yaw rotation (z-axis rotation)
  double siny_cosp =
      2 * (quat_in.w() * quat_in.z() + quat_in.x() * quat_in.y());
  double cosy_cosp =
      1 - 2 * (quat_in.y() * quat_in.y() + quat_in.z() * quat_in.z());
  double yaw = std::atan2(siny_cosp, cosy_cosp);

  // The following is the Eigen library method. But it flips for a negative
  // yaw..
  // Eigen::Matrix3d mat = quat_in.toRotationMatrix();
  // return mat.eulerAngles(2,1,0);

  return Eigen::Vector3d(yaw, pitch, roll);
}

// ZYX extrinsic rotation rates to world angular velocity
// angular vel = [wx, wy, wz]
Eigen::Vector3d EulerZYXRatestoAngVel(const double roll, const double pitch,
                                      const double yaw, const double roll_rate,
                                      const double pitch_rate,
                                      const double yaw_rate) {
  // From Robot Dynamics Lecture Notes - Robotic Systems Lab, ETH Zurich
  // Equation (2.86). The matrix has been reordered so that omega = E*[r;p;y]
  Eigen::Vector3d rpy_rates;
  rpy_rates << roll_rate, pitch_rate, yaw_rate;

  Eigen::MatrixXd E(3, 3);

  double y = pitch;
  double z = yaw;

  E << cos(y) * cos(z), -sin(z), 0, cos(y) * sin(z), cos(z), 0, -sin(y), 0, 1;

  return E * rpy_rates;
}

void avoid_quat_jump(const Eigen::Quaternion<double> &des_ori,
                     Eigen::Quaternion<double> &act_ori) {
  Eigen::Quaternion<double> ori_diff1;
  Eigen::Quaternion<double> ori_diff2;

  ori_diff1.w() = des_ori.w() - act_ori.w();
  ori_diff1.x() = des_ori.x() - act_ori.x();
  ori_diff1.y() = des_ori.y() - act_ori.y();
  ori_diff1.z() = des_ori.z() - act_ori.z();

  ori_diff2.w() = des_ori.w() + act_ori.w();
  ori_diff2.x() = des_ori.x() + act_ori.x();
  ori_diff2.y() = des_ori.y() + act_ori.y();
  ori_diff2.z() = des_ori.z() + act_ori.z();

  if (ori_diff1.squaredNorm() > ori_diff2.squaredNorm()) {
    act_ori.w() = -act_ori.w();
    act_ori.x() = -act_ori.x();
    act_ori.y() = -act_ori.y();
    act_ori.z() = -act_ori.z();
  } else
    act_ori = act_ori;
}

} // namespace myUtils
