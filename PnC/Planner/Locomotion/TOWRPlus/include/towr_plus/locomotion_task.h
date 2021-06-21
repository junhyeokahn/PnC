#pragma once

#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <towr_plus/models/endeffector_mappings.h>
#include <towr_plus/terrain/height_map.h>
#include <util/util.hpp>

using namespace towr_plus;

class LocomotionTask {
public:
  LocomotionTask(const std::string &name);
  virtual ~LocomotionTask();

  // ===========================================================================
  // Initial Configuration
  // ===========================================================================
  Eigen::VectorXd initial_base_lin; // 6D base linear pos, vel
  Eigen::VectorXd initial_base_ang; // 6D base angular pos, vel
  std::vector<Eigen::Vector3d> initial_ee_motion_lin; // 3D EE linear pos
  std::vector<Eigen::Vector3d> initial_ee_motion_ang; // 3D EE angular pos

  // ===========================================================================
  // Goal Configuration
  // ===========================================================================
  Eigen::VectorXd final_base_lin; // 6D base linear pos, vel
  Eigen::VectorXd final_base_ang; // 6D base angular pos, vel

  // ===========================================================================
  // Terrain
  // ===========================================================================
  std::shared_ptr<HeightMap> terrain;

  // ===========================================================================
  // Methods
  // ===========================================================================
  void to_one_hot_vector(
      const Eigen::VectorXd &one_hot_vec); // Initialize from one hot vector
  void print_info();                       // Print task information
  void from_yaml(const YAML::Node &node);  // Initialize from yaml

private:
  std::string name_;
};
