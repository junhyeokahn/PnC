#ifndef DCM_WALKING_PATTERN_GENERATOR_H
#define DCM_WALKING_PATTERN_GENERATOR_H

#include <PnC/DracoPnC/PredictionModule/DracoFootstep.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <string>
#include <iostream>


class DCMWalkingReference{
public:
  DCMWalkingReference();
  ~DCMWalkingReference(); 

  static int const DCM_SWING_VRP_TYPE;
  static int const DCM_TRANSFER_VRP_TYPE;
  std::vector<int> rvrp_type_list; // List of type of virtual repelant point

  std::vector<DracoFootstep> footstep_list; // Footstep list to generate pattern

  std::vector<Eigen::Vector3d> rvrp_list; // List of virtual repelant points.
  std::vector<Eigen::Vector3d> dcm_ini_list; // List of initial DCM states 
  std::vector<Eigen::Vector3d> dcm_eos_list; // List of end-of-step DCM states

  // Initial and Boundary Conditions for the continuous DS trajectory
  std::vector<Eigen::Vector3d> dcm_ini_DS_list; 
  std::vector<Eigen::Vector3d> dcm_vel_ini_DS_list; 
  std::vector<Eigen::Vector3d> dcm_end_DS_list; 
  std::vector<Eigen::Vector3d> dcm_vel_end_DS_list; 
  std::vector<Eigen::MatrixXd> dcm_P; // polynomial matrices for polynomial interpolation

  // DCM walking parameters
  double t_transfer = 0.1; // exponential interpolation transfer time during initial transfer or same step transfer
  double t_transfer_ds = 0.2; // polynomial transfer time time during initial transfer or same step transfer
  double t_ss = 0.3; // single support exponential interpolation transfer time
  double t_ds = 0.2; // double support polynomial transfer time during swing
  double alpha_ds = 0.5; // value between 0.0 and 1.0 for double support DCM interpolation

  void setCoMHeight(double z_vrp_in); // Sets the desired CoM Height
  void setInitialTime(double t_start_in); // Sets the initial offset time.


  // DCM trajectory calculation
  // input: input_footstep_list - a list of footsteps to take not including the current stance configuration.
  //        initial_footstance  - a footstep object describing the stance leg. 
  // populates this object's footstep_list, rvrp_list, dcm_ini_list, dcm_eos_list
  void initialize_footsteps_rvrp(const std::vector<DracoFootstep> & input_footstep_list, const DracoFootstep & initial_footstance, bool clear_list=false);

  // input: input_footstep_list - a list of footsteps to take not including the current stance configuration.
  //        initial_footstance  - a footstep object describing the stance leg. 
  //        initial_rvrp        - an initial virtual repelant point (eg: average of the stance feet's rvrp). 
  // populates this object's footstep_list, rvrp_list, dcm_ini_list, dcm_eos_list
  void initialize_footsteps_rvrp(const std::vector<DracoFootstep> & input_footstep_list, const DracoFootstep & initial_footstance, const Eigen::Vector3d & initial_rvrp);


  // input: input_footstep_list - a list of footsteps to take not including the current stance configuration.
  //        left_footstance        - a footstep object describing the left stance feet
  //        right_footstance       - a footstep object describing the right stance feet
  // populates this object's footstep_list, rvrp_list, dcm_ini_list, dcm_eos_list. 
  void initialize_footsteps_rvrp(const std::vector<DracoFootstep> & input_footstep_list, const DracoFootstep & left_footstance, const DracoFootstep & right_footstance);

  // input: input_footstep_list - a list of footsteps to take not including the current stance configuration.
  //        left_footstance        - a footstep object describing the left stance feet
  //        right_footstance       - a footstep object describing the right stance feet
  //        initial_com            - the initial location of the com 
  // populates this object's footstep_list, rvrp_list, dcm_ini_list, dcm_eos_list. 
  void initialize_footsteps_rvrp(const std::vector<DracoFootstep> & input_footstep_list, const DracoFootstep & left_footstance, const DracoFootstep & right_footstance, const Eigen::Vector3d & initial_com);


   // Outputs the average r_vrp location given two footstances
  void get_average_rvrp(const DracoFootstep & footstance_1, const DracoFootstep & footstance_2, Eigen::Vector3d & average_rvrp);

    // computes all the dcm states. Computation properly populates the dcm_ini_list and dcm_eos_list
  void computeDCM_states();

  // Compute: DCM, DCM vel, CoM Vel, CoM, given time, t.



private:
  // DCM parameters:
  double gravity = 9.81;
  double z_vrp = 0.75; // desired VRP height / CoM height
  double b = std::sqrt(z_vrp/gravity); // time constant of DCM dynamics  

  double t_start = 0.0; // the starting time for the DCM Walking reference

  // input: r_vrp_d_i - the desired virtual repelant point for the i-th step.
  //        t_step    - the time interval to use for backwards integration
  //        dcm_eos_i - the DCM state at the end of the i-th step. 
  // computes the step i's initial DCM state and the end-of-step i-1's dcm state. 
  // The computation is stored in the dcm_ini_list and dcm_eos_list. 
  Eigen::Vector3d computeDCM_ini_i(const Eigen::Vector3d & r_vrp_d_i, const double & t_step, const Eigen::Vector3d & dcm_eos_i);

  // Computes the double support DCM boundary conditions for continuous double support trajectories
  // step_index the boundary condition for the step index
  // t_DS_{ini, end} the interpolation duration away from a Virtual repelant point. 
  Eigen::Vector3d computeDCM_iniDS_i(const int & step_index, const double t_DS_ini);
  Eigen::Vector3d computeDCM_eoDS_i(const int & step_index, const double t_DS_end);
  Eigen::Vector3d computeDCMvel_iniDS_i(const int & step_index, const double t_DS_ini);
  Eigen::Vector3d computeDCMvel_eoDS_i(const int & step_index, const double t_DS_end);

  // Returns the DCM exponential interpolation for the requested step_index.
  // time, t, is clamped between 0.0 and t_step.
  Eigen::Vector3d get_DCM_exp(const int & step_index, const double & t);

  // Returns the DCM velocity exponential interpolation for the requested step_index.
  // time, t, is clamped between 0.0 and t_step.
  Eigen::Vector3d get_DCM_vel_exp(const int & step_index, const double & t);


  // Returns the DCM double support polynomial interpolation for the requested step_index.
  // time, t, is clamped between 0.0 and t_step.
  Eigen::Vector3d get_DCM_DS_poly(const int & step_index, const double & t);

  // Returns the DCM double support velocity polynomial interpolation for the requested step_index.
  // time, t, is clamped between 0.0 and t_step.
  Eigen::Vector3d get_DCM_DS_vel_poly(const int & step_index, const double & t);


  // Get the t_step for step i.
  double get_t_step(const int & step_i);

  // returns matrix P \in R^{4x3}
  // P = mat_coeff * [dcm_ini^T; \dot{dcm}_ini^T; dcm_end^T; \dot{dcm}_end]
  // where mat_coeff \in R^{4x4} and ^T denotes a vector transpose transpose.
  // Ts is the duration in seconds
  Eigen::MatrixXd polynomialMatrix(const double Ts,
                                   const Eigen::Vector3d & dcm_ini, const Eigen::Vector3d & dcm_vel_ini,
                                   const Eigen::Vector3d & dcm_end, const Eigen::Vector3d & dcm_vel_end);


};

#endif