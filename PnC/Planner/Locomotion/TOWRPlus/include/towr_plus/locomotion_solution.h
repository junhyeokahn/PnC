#pragma once

#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <util/util.hpp>

#include <towr_plus/models/endeffector_mappings.h>
#include <towr_plus/variables/cartesian_dimensions.h>
#include <towr_plus/variables/nodes_variables_all.h>
#include <towr_plus/variables/phase_durations.h>
#include <towr_plus/variables/spline_holder.h>
#include <towr_plus/variables/variable_names.h>

using namespace towr_plus;

// Usage: initialize --> from_one_hot_vector
class LocomotionSolution {
public:
  LocomotionSolution(const std::string &name, const YAML::Node &node);
  virtual ~LocomotionSolution();

  // ===========================================================================
  // Methods
  // ===========================================================================
  void from_one_hot_vector(
      const Eigen::VectorXd &one_hot_vec); // Initialize nodes and splines from
                                           // one hot vector
  void print_info();                       // Print solution information
  void print_solution(double dt = 0.05);   // Print solution
  void to_yaml(double dt = 0.01);          // Save solution to yaml

private:
  std::string name_;

  // ===========================================================================
  // Planning Parameters to Initialize Variables
  // ===========================================================================
  double duration_base_polynomial_;
  int force_polynomials_per_stance_phase_;
  int ee_polynomials_per_swing_phase_; // Assume this is always 2
  bool b_optimize_timings_;
  std::vector<std::vector<double>> ee_phase_durations_;

  // ===========================================================================
  // For Constructing Splines
  // ===========================================================================
  int parsing_idx_;
  Eigen::VectorXd one_hot_vector_;
  Eigen::VectorXd one_hot_base_lin_;
  Eigen::VectorXd one_hot_base_ang_;
  std::vector<Eigen::VectorXd> one_hot_ee_motion_lin_;
  std::vector<Eigen::VectorXd> one_hot_ee_motion_ang_;
  std::vector<Eigen::VectorXd> one_hot_ee_wrench_lin_;
  std::vector<Eigen::VectorXd> one_hot_ee_wrench_ang_;
  std::vector<Eigen::VectorXd> one_hot_ee_contact_schedule_;

  SplineHolder spline_holder_;
  // ===========================================================================
  // For Saving
  // ===========================================================================
  int n_base_nodes_;
  int n_base_vars_;
  Eigen::MatrixXd base_lin_nodes_;
  Eigen::MatrixXd base_ang_nodes_;

  std::vector<int> n_ee_motion_nodes_;
  std::vector<int> n_ee_motion_lin_vars_;
  std::vector<int> n_ee_motion_ang_vars_;
  std::vector<Eigen::MatrixXd> ee_motion_lin_nodes_;
  std::vector<Eigen::MatrixXd> ee_motion_ang_nodes_;

  std::vector<int> n_ee_wrench_nodes_;
  std::vector<int> n_ee_wrench_vars_;
  std::vector<Eigen::MatrixXd> ee_wrench_lin_nodes_;
  std::vector<Eigen::MatrixXd> ee_wrench_ang_nodes_;

  std::vector<std::vector<double>> ee_schedules_;

  // ===========================================================================
  // Helper Functions to Initialize Variables
  // ===========================================================================
  std::vector<double> _get_base_poly_duration();
  double _get_total_time();
  Eigen::MatrixXd _transpose(Eigen::MatrixXd mat, std::vector<int> order,
                             std::string col_or_row);

  // ===========================================================================
  // Methods for Parsing One Hot Encoded Solution
  // ===========================================================================
  void _set_base_nodes();
  void _set_ee_motion_nodes();
  void _set_ee_wrench_nodes();
  void _set_ee_schedule_variables();
  void _set_splines();
};
