#pragma once

#include <iostream>

#include "utils/util.hpp"

/// class JointIntegrator
class JointIntegrator {
public:
  /// \{ \name Constructor and Destructor
  JointIntegrator(int num_joints_in, double dt_in);

  JointIntegrator(const int num_joints_in, const double vel_cutoff_in,
                  const double pos_cutoff_in, const double dt_in);

  ~JointIntegrator();
  /// \}

  /// Initialize internal starting velocity and position states
  void initializeStates(const Eigen::VectorXd init_vel,
                        const Eigen::VectorXd init_pos);

  /// Performs a leaky integration on the velocity and position.
  void integrate(const Eigen::VectorXd acc_in, const Eigen::VectorXd &vel_in,
                 const Eigen::VectorXd &pos_in, Eigen::VectorXd &vel_out,
                 Eigen::VectorXd &pos_out);

  /// Set time step.
  void setDt(const double dt_in);

  /// Set velocity cutoff frequency.
  /// \note Set this value to 0.0 to perform traditional integration
  void setVelocityFrequencyCutOff(const double vel_cutoff_in);

  /// Set position cutoff frequency.
  /// \note Set this value to 0.0 to perform traditional integration
  void setPositionFrequencyCutOff(const double pos_cutoff_in);

  /// Set joint velocity limits
  void setVelocityBounds(const Eigen::VectorXd vel_min_in,
                         const Eigen::VectorXd vel_max_in);

  /// Set joint positionlimits
  void setPositionBounds(const Eigen::VectorXd pos_min_in,
                         const Eigen::VectorXd pos_max_in);

  /// Set the maximum position deviation from current position
  void setMaxPositionError(const double pos_max_error_in);

  /// Set the maximum position deviation from current position
  void setMaxPositionErrorVector(const Eigen::VectorXd pos_max_error_in);

  /// Print outs for debugging purpose.
  void printIntegrationParams();

  /// Whether it is initialized or not.
  bool isInitialized() { return b_initialized; };

private:
  /// Number of joints.
  int n_joints_;

  /// Velocity cutoff frequency.
  double vel_freq_cutoff_;

  /// Position cutoff frequency.
  double pos_freq_cutoff_;

  /// Equivalent alpha cut-off for velocity integration
  double alpha_vel_;

  /// Equivalent alpha cut-off for positionintegration
  double alpha_pos_;

  /// Integration timestep
  double dt_;

  /// Boolean indicator whether it is initialized or not
  bool b_initialized;

  // Internal velocities.
  Eigen::VectorXd vel_;

  // Internal positions.
  Eigen::VectorXd pos_;

  Eigen::VectorXd vel_min_;
  Eigen::VectorXd vel_max_;

  Eigen::VectorXd pos_min_;
  Eigen::VectorXd pos_max_;
  Eigen::VectorXd pos_max_error_;

  double default_vel_freq_cutoff_ = 2.0; // Hz
  double default_pos_freq_cutoff_ = 1.0; // Hz

  void setDefaultSaturation();
  double default_vel_min_max_ = 2;        // +/- radians/s
  double default_pos_min_max_ = 2 * M_PI; //  +/- radians
  double default_pos_max_error_ =
      0.2; // radians maximum position deviation from actual

  /// Ouptuts alpha between 0 and 1 from a set frequency and dt
  double getAlphaFromFrequency(const double hz, const double dt);
  /// Clamps a value to be within min and max bounds
  double clampValue(const double in, const double min, const double max);
  /// Clamps a vector value to be within min and max bounds.
  Eigen::VectorXd clampVec(const Eigen::VectorXd vec_in,
                           const Eigen::VectorXd vec_min,
                           const Eigen::VectorXd vec_max);
};
