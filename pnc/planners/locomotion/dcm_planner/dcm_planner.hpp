#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <map>
#include <string>

#include "pnc/planners/locomotion/dcm_planner/footstep.hpp"
#include "utils/interpolation.hpp"
#include "utils/util.hpp"

/// namespace vrp_type
namespace vrp_type {
constexpr int kRFootSwing = 1;
constexpr int kLFootSwing = 2;
constexpr int kTransfer = 3;
constexpr int kEnd = 4;
} // namespace vrp_type

/// class DCMPlanner
class DCMPlanner {
public:
  /// \{ \name Constructor and Destructor
  DCMPlanner();
  ~DCMPlanner();
  /// \}

  void paramInitialization(const YAML::Node &node);
  /// Set robot mass.
  void setRobotMass(double mass);
  /// Set COM Height.
  void setCoMHeight(double z_vrp_in);
  /// Set start time.
  void setInitialTime(double t_start_in);
  /// Set initial orientation.
  void setInitialOri(const Eigen::Quaterniond initial_ori_in);
  /// Returns start time;
  double getInitialTime();
  /// DCM planning given footstep list input.
  void
  initialize_footsteps_rvrp(const std::vector<Footstep> &input_footstep_list,
                            const Footstep &initial_footstance,
                            bool clear_list = false);

  /// DCM planning given footstep list input.
  void
  initialize_footsteps_rvrp(const std::vector<Footstep> &input_footstep_list,
                            const Footstep &initial_footstance,
                            const Eigen::Vector3d &initial_rvrp);

  /// DCM planning given footstep list input.
  void
  initialize_footsteps_rvrp(const std::vector<Footstep> &input_footstep_list,
                            const Footstep &left_footstance,
                            const Footstep &right_footstance);

  /// DCM planning given footstep list input.
  void
  initialize_footsteps_rvrp(const std::vector<Footstep> &input_footstep_list,
                            const Footstep &left_footstance,
                            const Footstep &right_footstance,
                            const Eigen::Vector3d &initial_com);

  /// DCM planning given footstep list input.
  void
  initialize_footsteps_rvrp(const std::vector<Footstep> &input_footstep_list,
                            const Footstep &left_footstance,
                            const Footstep &right_footstance,
                            const Eigen::Vector3d &initial_dcm,
                            const Eigen::Vector3d &initial_dcm_vel);

  /// List of type of virtual repelant point.
  std::vector<int> rvrp_type_list;
  /// Footstep list to generate pattern.
  std::vector<Footstep> footstep_list;
  /// List of VRPs.
  std::vector<Eigen::Vector3d> rvrp_list;
  /// List of initial DCM states.
  std::vector<Eigen::Vector3d> dcm_ini_list;
  /// List of end-of-step DCM states.
  std::vector<Eigen::Vector3d> dcm_eos_list;

  /// Initial DCM position.
  Eigen::Vector3d ini_dcm_pos;
  /// Initial DCM velocity.
  Eigen::Vector3d ini_dcm_vel;

  /// Map containing the rvrp index that have a corresponding footstep swing.
  std::map<int, int> rvrp_index_to_footstep_index;

  /// Vector containing hermite quaternion curve objects for the pelvis
  /// orientation.
  std::vector<HermiteQuaternionCurve> pelvis_ori_quat_curves;

  /// \{ \name Initial and Boundary Conditions
  std::vector<Eigen::Vector3d> dcm_ini_DS_list;
  std::vector<Eigen::Vector3d> dcm_vel_ini_DS_list;
  std::vector<Eigen::Vector3d> dcm_acc_ini_DS_list;
  std::vector<Eigen::Vector3d> dcm_end_DS_list;
  std::vector<Eigen::Vector3d> dcm_vel_end_DS_list;
  std::vector<Eigen::Vector3d> dcm_acc_end_DS_list;
  /// polynomial matrices for polynomial interpolation
  std::vector<Eigen::MatrixXd> dcm_P;
  /// minjerk curves for interpolation
  std::vector<MinJerkCurveVec> dcm_minjerk;
  /// \}

  /// \{ \name DCM Planning Parameters
  /// Exponential interpolation transfer time during initial transfer or same
  /// step transfer.
  double t_transfer = 0.1;
  /// double support polynomial transfer time.
  double t_ds = 0.05;
  /// single support exponential interpolation time.
  double t_ss = 0.3;
  /// percent to converge at the end of the trajectory.
  double percentage_settle = 0.99;
  /// value between 0.0 and 1.0 for double support DCM interpolation
  double alpha_ds = 0.5;
  /// \}

  /// Get desired DCM position.
  void get_ref_dcm(const double t, Eigen::Vector3d &dcm_out);

  /// Get desired DCM velocity.
  void get_ref_dcm_vel(const double t, Eigen::Vector3d &dcm_vel_out);

  /// Get desired COM position.
  void get_ref_com(const double t, Eigen::Vector3d &com_out);

  /// Get desired COM velocity.
  void get_ref_com_vel(const double t, Eigen::Vector3d &com_vel_out);

  /// Get desired COM acceleration.
  void get_ref_com_acc(const double t, Eigen::Vector3d &com_acc_out);

  /// Get desired VRP.
  void get_ref_r_vrp(const double t, Eigen::Vector3d &r_vrvp_out);

  /// Get desired reaction force.
  void get_ref_reaction_force(const double t, Eigen::Vector3d &f_out);

  /// Get desired base orientation pos, vel, acc.
  void get_ref_ori_ang_vel_acc(const double t, Eigen::Quaterniond &quat_out,
                               Eigen::Vector3d &ang_vel_out,
                               Eigen::Vector3d &ang_acc_out);

  /// Computes CoM velocity given the current CoM position and DCM velocity
  /// state.
  void get_com_vel(const Eigen::Vector3d &com_pos, const Eigen::Vector3d &dcm,
                   Eigen::Vector3d &com_vel_out);

  /// Computes CoM acceleration given the current CoM position and DCM velocity
  /// state.
  void get_com_acc(const Eigen::Vector3d &com_vel,
                   const Eigen::Vector3d &dcm_vel,
                   Eigen::Vector3d &com_acc_out);

  /// Computes the CoM reaction force / leg reaction force given mass, CoM
  /// position and the r_vrp
  void get_reaction_force(const double mass, const Eigen::Vector3d &com_pos,
                          const Eigen::Vector3d &r_vrp, Eigen::Vector3d fr_out);
  /// computes the current r_vrp given
  void get_r_vrp(const double b_in, const Eigen::Vector3d &dcm,
                 const Eigen::Vector3d &dcm_vel, Eigen::Vector3d &r_vrp_out);

  /// Print out boundary conditions of the DCM
  void printBoundaryConditions();

  /// Returns the exponential step index the current time falls in.
  int which_step_index(const double t);
  /// Returns the polynomial step index to use given the input time from
  /// t_start.
  int which_step_index_to_use(const double t);

  /// returns the start time of the step_index from t_start.
  double get_t_step_start(const int step_index);
  /// returns the end time of the step_index from t_start.
  double get_t_step_end(const int step_index);

  /// returns the double support start time of the step_index from t_start.
  double get_double_support_t_start(const int step_index);
  /// returns the double support end time of the step_index from t_start.
  double get_double_support_t_end(const int step_index);

  /// If the step_index is a swing type, return true an populate the swing
  /// start time and end.
  bool get_t_swing_start_end(const int step_index, double &swing_start_time,
                             double &swing_end_time);

  /// Returns the polynomial duration for the given step index
  double get_polynomial_duration(const int step_index);

  /// Get the end of double support transition times
  double get_eoDS_transition_time();
  /// Get the initial double support transition times
  double get_iniDS_transition_time();

  /// Return the type of VRP. Clamps the index to valid values.
  int get_r_vrp_type(const int step_index);

  /// Returns t_end
  double get_total_trajectory_time();

  /// Returns t_settle;
  double get_settle_time() {
    double t_settle = -b * log(1.0 - percentage_settle);
    return t_settle;
  }

private:
  /// Get the t_step for step i.
  double get_t_step(const int &step_i);

  /// Compute the step i's initial DCM state and the end-of-step i-1's dcm
  /// state.
  Eigen::Vector3d computeDCM_ini_i(const Eigen::Vector3d &r_vrp_d_i,
                                   const double &t_step,
                                   const Eigen::Vector3d &dcm_eos_i);

  /// Compute the double support DCM boundary conditions for continuous double
  /// support trajectories
  Eigen::Vector3d computeDCM_iniDS_i(const int &step_index,
                                     const double t_DS_ini);
  /// Compute the double support DCM boundary conditions for continuous double
  /// support trajectories
  Eigen::Vector3d computeDCM_eoDS_i(const int &step_index,
                                    const double t_DS_end);
  /// Compute the double support DCM boundary conditions for continuous double
  /// support trajectories
  Eigen::Vector3d computeDCMvel_iniDS_i(const int &step_index,
                                        const double t_DS_ini);
  /// Compute the double support DCM boundary conditions for continuous double
  /// support trajectories
  Eigen::Vector3d computeDCMvel_eoDS_i(const int &step_index,
                                       const double t_DS_end);
  /// Compute the double support DCM boundary conditions for continuous double
  /// support trajectories
  Eigen::Vector3d computeDCMacc_iniDS_i(const int &step_index,
                                        const double t_DS_ini);
  /// Compute the double support DCM boundary conditions for continuous double
  /// support trajectories
  Eigen::Vector3d computeDCMacc_eoDS_i(const int &step_index,
                                       const double t_DS_end);

  double robot_mass = 50;
  double gravity = 9.81;
  double z_vrp = 0.75;
  double b = std::sqrt(z_vrp / gravity);

  double t_start = 0.0;

  /// Return the average r_vrp location given two footstances.
  void get_average_rvrp(const Footstep &footstance_1,
                        const Footstep &footstance_2,
                        Eigen::Vector3d &average_rvrp);

  /// Compute all the dcm states. Computation should properly populate the
  /// dcm_ini_list and dcm_eos_list
  void computeDCM_states();

  /// Sums up the total trajectory time and stores the result in t_end.
  void compute_total_trajectory_time();
  double t_end = 0.0;

  /// computes the reference com trajectories by integration.
  void compute_reference_com();
  /// Discretization factor used.
  double dt_local = 1e-3;
  /// Containers for the integrated reference CoM position.
  std::vector<Eigen::Vector3d> ref_com_pos;
  /// Containers for the integrated reference CoM velocity.
  std::vector<Eigen::Vector3d> ref_com_vel;
  /// Containers for the integrated reference CoM acceleration.
  std::vector<Eigen::Vector3d> ref_com_acc;

  /// Computes the reference pelvis orientation.
  void compute_reference_pelvis_ori();
  Footstep initial_leftfoot_stance;
  Footstep initial_rightfoot_stance;
  Eigen::Quaterniond initial_ori;

  /// Return the DCM exponential interpolation for the requested step_index.
  /// time, t, is clamped between 0.0 and t_step.
  Eigen::Vector3d get_DCM_exp(const int &step_index, const double &t);

  /// Return the DCM velocity exponential interpolation for the requested
  /// step_index.
  Eigen::Vector3d get_DCM_vel_exp(const int &step_index, const double &t);

  /// Return the DCM acceleration exponential interpolation for the requested
  /// step_index.
  Eigen::Vector3d get_DCM_acc_exp(const int &step_index, const double &t);

  /// Return the DCM double support polynomial interpolation for the requested
  /// step_index.
  Eigen::Vector3d get_DCM_DS_poly(const int &step_index, const double &t);

  /// Return the DCM double support velocity polynomial interpolation for the
  /// requested step_index.
  Eigen::Vector3d get_DCM_DS_vel_poly(const int &step_index, const double &t);

  /// Return the DCM double support min jerk interpolation for the requested
  /// step_index.
  Eigen::Vector3d get_DCM_DS_minjerk(const int &step_index, const double &t);

  /// Return the DCM double support velocity min jerk interpolation for the
  /// requested step_index.
  Eigen::Vector3d get_DCM_DS_vel_minjerk(const int &step_index,
                                         const double &t);

  /// Return matrix P
  Eigen::MatrixXd polynomialMatrix(const double Ts,
                                   const Eigen::Vector3d &dcm_ini,
                                   const Eigen::Vector3d &dcm_vel_ini,
                                   const Eigen::Vector3d &dcm_end,
                                   const Eigen::Vector3d &dcm_vel_end);

  int clampINT(int input, int low_bound, int upper_bound);
  double clampDOUBLE(double input, double low_bound, double upper_bound);
};
