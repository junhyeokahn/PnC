#pragma once

#include <algorithm>
#include <fstream>
#include <iostream>
#include <list>
#include <stdio.h>

#include <Eigen/Dense>
#include <Eigen/QR>

#include "configuration.hpp"
#include "third_party/yaml/include/yaml/yaml.h"

namespace color {
constexpr int kRed = 0;
constexpr int kBoldRed = 1;
constexpr int kGreen = 2;
constexpr int kBoldGreen = 3;
constexpr int kYellow = 4;
constexpr int kBoldYellow = 5;
constexpr int kBlue = 6;
constexpr int kBoldBlue = 7;
constexpr int kMagneta = 8;
constexpr int kBoldMagneta = 9;
constexpr int kCyan = 10;
constexpr int kBoldCyan = 11;
}; // namespace color

namespace util {
void SaveVector(const Eigen::VectorXd &vec_, std::string name_,
                bool b_param = false);
void SaveVector(double *_vec, std::string _name, int size,
                bool b_param = false);
void SaveVector(const std::vector<double> &_vec, std::string _name,
                bool b_param = false);
void SaveValue(double _value, std::string _name, bool b_param = false);
void CleaningFile(std::string file_name_, std::string &ret_file_, bool b_param);
static std::list<std::string> gs_fileName_string; // global & static

template <typename YamlType>
YamlType ReadParameter(const YAML::Node &node, const std::string &name) {
  try {
    return node[name.c_str()].as<YamlType>();
  } catch (...) {
    throw std::runtime_error(name);
  }
};

template <typename YamlType>
void ReadParameter(const YAML::Node &node, const std::string &name,
                   YamlType &parameter) {
  try {
    parameter = ReadParameter<YamlType>(node, name);
  } catch (...) {
    throw std::runtime_error(name);
  }
};

void PrettyConstructor(const int &_num_tab, const std::string &_name);
void ColorPrint(const int &_color, const std::string &_name,
                bool line_change = true);

Eigen::MatrixXd hStack(const Eigen::MatrixXd &a_, const Eigen::MatrixXd &b_);
Eigen::MatrixXd vStack(const Eigen::MatrixXd &a_, const Eigen::MatrixXd &b_);
Eigen::MatrixXd block_diag(const Eigen::MatrixXd &a, const Eigen::MatrixXd &b);

Eigen::Matrix3d SkewSymmetric(const Eigen::Vector3d &omg);
Eigen::MatrixXd Adjoint(const Eigen::MatrixXd &R, const Eigen::Vector3d &p);

Eigen::Vector3d QuatToExp(const Eigen::Quaternion<double> &quat);
Eigen::Quaternion<double> ExpToQuat(const Eigen::Vector3d &exp);

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
                                  const double yaw);

// Quaternion to Euler ZYX
Eigen::Vector3d QuatToEulerZYX(const Eigen::Quaterniond &quat_in);

// ZYX extrinsic rotation rates to world angular velocity
// angular vel = [wx, wy, wz]
Eigen::Vector3d EulerZYXRatestoAngVel(const double roll, const double pitch,
                                      const double yaw, const double roll_rate,
                                      const double pitch_rate,
                                      const double yaw_rate);

void AvoidQuatJump(const Eigen::Quaternion<double> &des_ori,
                   Eigen::Quaternion<double> &act_ori);

double Clamp(const double &s_in, double lo = 0.0, double hi = 1.0);

void PseudoInverse(Eigen::MatrixXd const &matrix, double sigmaThreshold,
                   Eigen::MatrixXd &invMatrix,
                   Eigen::VectorXd *opt_sigmaOut = 0);

Eigen::MatrixXd PseudoInverse(const Eigen::MatrixXd &matrix,
                              const double &threshold);

Eigen::MatrixXd NullSpace(const Eigen::MatrixXd &J,
                          const double threshold = 0.00001);

Eigen::MatrixXd WeightedPseudoInverse(const Eigen::MatrixXd &J,
                                      const Eigen::MatrixXd &W,
                                      const double sigma_threshold = 0.0001);
} // namespace util
