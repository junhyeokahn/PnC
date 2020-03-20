#ifndef DCM_WALKING_PATTERN_GENERATOR_H
#define DCM_WALKING_PATTERN_GENERATOR_H

#include <PnC/DracoPnC/PredictionModule/DracoFootstep.hpp>
#include <Utils/Math/minjerk_vec.hpp>
#include <Utils/Math/hermite_quaternion_curve.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <string>
#include <iostream>
#include <map>

class DCMWalkingReference{
public:
  DCMWalkingReference();
  ~DCMWalkingReference(); 

  static int const DCM_RL_SWING_VRP_TYPE;
  static int const DCM_LL_SWING_VRP_TYPE;
  static int const DCM_MIDSTEP_TRANSFER_VRP_TYPE;
  static int const DCM_TRANSFER_VRP_TYPE;
  static int const DCM_END_VRP_TYPE;

  std::vector<int> rvrp_type_list; // List of type of virtual repelant point

  std::vector<DracoFootstep> footstep_list; // Footstep list to generate pattern

  std::vector<Eigen::Vector3d> rvrp_list; // List of virtual repelant points.
  std::vector<Eigen::Vector3d> dcm_ini_list; // List of initial DCM states 
  std::vector<Eigen::Vector3d> dcm_eos_list; // List of end-of-step DCM states

  // Initial DCM states
  Eigen::Vector3d ini_dcm_pos;
  Eigen::Vector3d ini_dcm_vel;

  // map containing the rvrp index that have a corresponding footstep swing
  std::map<int, int> rvrp_index_to_footstep_index;
  // Vector containing hermite quaternion curve objects for the pelvis orientation.
  // one curve per RVRP.
  std::vector<HermiteQuaternionCurve> pelvis_ori_quat_curves;

  // Initial and Boundary Conditions for the continuous DS trajectory
  std::vector<Eigen::Vector3d> dcm_ini_DS_list; 
  std::vector<Eigen::Vector3d> dcm_vel_ini_DS_list; 
  std::vector<Eigen::Vector3d> dcm_acc_ini_DS_list; 
  std::vector<Eigen::Vector3d> dcm_end_DS_list; 
  std::vector<Eigen::Vector3d> dcm_vel_end_DS_list; 
  std::vector<Eigen::Vector3d> dcm_acc_end_DS_list;  
  std::vector<Eigen::MatrixXd> dcm_P; // polynomial matrices for polynomial interpolation
  std::vector<MinJerkCurveVec> dcm_minjerk; // minjerk curves for interpolation

  // DCM walking parameters
  double t_transfer = 0.1; //0.125 ; //0.1; // exponential interpolation transfer time during initial transfer or same step transfer
  double t_ds = 0.05; // double support polynomial transfer time
  double t_ss = 0.3; // single support exponential interpolation  time
  double t_midstep_transfer = 0.15;

  double percentage_settle = 0.99;//0.999; // percent to converge at the end of the trajectory

  double alpha_ds = 0.5; // value between 0.0 and 1.0 for double support DCM interpolation

  void setRobotMass(double mass);
  void setCoMHeight(double z_vrp_in); // Sets the desired CoM Height
  void setInitialTime(double t_start_in); // Sets the initial offset time.
  void setInitialOri(const Eigen::Quaterniond initial_ori_in);

  double getInitialTime(); // Returns t_start;


  // DCM trajectory calculation
  // input: input_footstep_list - a list of footsteps to take not including the current stance configuration.
  //        initial_footstance  - a footstep object describing the stance leg. 
  void initialize_footsteps_rvrp(const std::vector<DracoFootstep> & input_footstep_list, const DracoFootstep & initial_footstance, bool clear_list=false);

  // input: input_footstep_list - a list of footsteps to take not including the current stance configuration.
  //        initial_footstance  - a footstep object describing the stance leg. 
  //        initial_rvrp        - an initial virtual repelant point (eg: average of the stance feet's rvrp). 
  void initialize_footsteps_rvrp(const std::vector<DracoFootstep> & input_footstep_list, const DracoFootstep & initial_footstance, const Eigen::Vector3d & initial_rvrp);


  // input: input_footstep_list - a list of footsteps to take not including the current stance configuration.
  //        left_footstance        - a footstep object describing the left stance feet
  //        right_footstance       - a footstep object describing the right stance feet
  void initialize_footsteps_rvrp(const std::vector<DracoFootstep> & input_footstep_list, const DracoFootstep & left_footstance, const DracoFootstep & right_footstance);

  // input: input_footstep_list - a list of footsteps to take not including the current stance configuration.
  //        left_footstance        - a footstep object describing the left stance feet
  //        right_footstance       - a footstep object describing the right stance feet
  //        initial_com            - the initial location of the com 
  void initialize_footsteps_rvrp(const std::vector<DracoFootstep> & input_footstep_list, const DracoFootstep & left_footstance, const DracoFootstep & right_footstance, const Eigen::Vector3d & initial_com);

  // input: input_footstep_list - a list of footsteps to take not including the current stance configuration.
  //        left_footstance        - a footstep object describing the left stance feet
  //        right_footstance       - a footstep object describing the right stance feet
  //        initial_dcm            - the initial starting position of the dcm
  //        initial_dcm_vel        - the initial starting velocity of the dcm
  void initialize_footsteps_rvrp(const std::vector<DracoFootstep> & input_footstep_list, 
                                 const DracoFootstep & left_footstance, const DracoFootstep & right_footstance, 
                                 const Eigen::Vector3d & initial_dcm, const Eigen::Vector3d & initial_dcm_vel);


  // Compute: DCM, DCM vel, CoM, CoM Vel, given time, t.
  // t is a global time.
  void get_ref_dcm(const double t, Eigen::Vector3d & dcm_out);
  void get_ref_dcm_vel(const double t, Eigen::Vector3d & dcm_vel_out);
  void get_ref_com(const double t, Eigen::Vector3d & com_out);
  void get_ref_com_vel(const double t, Eigen::Vector3d & com_vel_out);
  void get_ref_r_vrp(const double t, Eigen::Vector3d & r_vrvp_out);
  void get_ref_reaction_force(const double t, Eigen::Vector3d & f_out);

  // Global reference quat, ang vel and ang acc
  void get_ref_ori_ang_vel_acc(const double t, Eigen::Quaterniond & quat_out,
                                               Eigen::Vector3d & ang_vel_out,
                                               Eigen::Vector3d & ang_acc_out);

  // computes the CoM velocity given the current CoM position and DCM velocity state.
  void get_com_vel(const Eigen::Vector3d & com_pos, const Eigen::Vector3d & dcm, Eigen::Vector3d & com_vel_out);
  // computes the CoM reaction force / leg reaction force given mass, CoM position and the r_vrp
  void get_reaction_force(const double mass, const Eigen::Vector3d & com_pos, const Eigen::Vector3d & r_vrp, Eigen::Vector3d fr_out);
  // computes the current r_vrp given 
  //   -the DCM dynamics time constant b,
  //  - the current dcm 
  //  - and the current dcm_vel
  void get_r_vrp(const double b_in, const Eigen::Vector3d & dcm, const Eigen::Vector3d & dcm_vel, Eigen::Vector3d & r_vrp_out);

  // prints the boundary conditions of the DCM
  void printBoundaryConditions();

  // Helper Functions
  // Returns the exponential step index the current time falls in.
  int which_step_index(const double t);
  // Returns the polynomial step index to use given the input time from t_start.
  int which_step_index_to_use(const double t);

  // returns the starting and ending time of the step_index from t_start.
  double get_t_step_start(const int step_index);
  double get_t_step_end(const int step_index);

  // returns the double support starting and ending time of the step_index from t_start.
  double get_double_support_t_start(const int step_index);
  double get_double_support_t_end(const int step_index);

  // if the step_index is a swing type, returns true an populates the swing start time and end
  bool get_t_swing_start_end(const int step_index, double & swing_start_time, double & swing_end_time);

  // returns the polynomial duration for the given step index
  double get_polynomial_duration(const int step_index);

  // Get the initial and end of double support transition times
  double get_eoDS_transition_time();
  double get_iniDS_transition_time();  

  // Returns the type of VRP. Clamps the index to valid values.
  int get_r_vrp_type(const int step_index);

  // Returns t_end
  double get_total_trajectory_time();

private:
  // DCM parameters:
  double robot_mass = 50; // kg
  double gravity = 9.81;
  double z_vrp = 0.75; // desired VRP height / CoM height
  double b = std::sqrt(z_vrp/gravity); // time constant of DCM dynamics  

  double t_start = 0.0; // the starting time for the DCM Walking reference

   // Outputs the average r_vrp location given two footstances
  void get_average_rvrp(const DracoFootstep & footstance_1, const DracoFootstep & footstance_2, Eigen::Vector3d & average_rvrp);

    // computes all the dcm states. Computation properly populates the dcm_ini_list and dcm_eos_list
  void computeDCM_states();

  // Sums up the total trajectory time and stores the result in t_end
  void compute_total_trajectory_time();
  double t_end = 0.0;

  // computes the reference com trajectories by integration
  void compute_reference_com();
  double dt_local = 1e-3; // discretization factor used 
  // containers for the integrated reference CoM position and velocites
  std::vector<Eigen::Vector3d> ref_com_pos;
  std::vector<Eigen::Vector3d> ref_com_vel;

  // computes the reference pelvis orientation.
  void compute_reference_pelvis_ori();
  DracoFootstep initial_leftfoot_stance;
  DracoFootstep initial_rightfoot_stance;
  Eigen::Quaterniond initial_ori;


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
  Eigen::Vector3d computeDCMacc_iniDS_i(const int & step_index, const double t_DS_ini);
  Eigen::Vector3d computeDCMacc_eoDS_i(const int & step_index, const double t_DS_end);

  // Returns the DCM exponential interpolation for the requested step_index.
  // time, t, is clamped between 0.0 and t_step.
  Eigen::Vector3d get_DCM_exp(const int & step_index, const double & t);

  // Returns the DCM velocity exponential interpolation for the requested step_index.
  // time, t, is clamped between 0.0 and t_step.
  Eigen::Vector3d get_DCM_vel_exp(const int & step_index, const double & t);

  // Returns the DCM acceleration exponential interpolation for the requested step_index.
  // time, t, is clamped between 0.0 and t_step.
  Eigen::Vector3d get_DCM_acc_exp(const int & step_index, const double & t);

  // Returns the DCM double support polynomial interpolation for the requested step_index.
  // time, t, is clamped between 0.0 and t_step.
  Eigen::Vector3d get_DCM_DS_poly(const int & step_index, const double & t);

  // Returns the DCM double support velocity polynomial interpolation for the requested step_index.
  // time, t, is clamped between 0.0 and t_step.
  Eigen::Vector3d get_DCM_DS_vel_poly(const int & step_index, const double & t);


  // Returns the DCM double support min jerk interpolation for the requested step_index.
  // time, t, is clamped between 0.0 and t_step.
  Eigen::Vector3d get_DCM_DS_minjerk(const int & step_index, const double & t);

  // Returns the DCM double support velocity min jerk interpolation for the requested step_index.
  // time, t, is clamped between 0.0 and t_step.
  Eigen::Vector3d get_DCM_DS_vel_minjerk(const int & step_index, const double & t);


  // Get the t_step for step i.
  double get_t_step(const int & step_i);

  // returns matrix P \in R^{4x3}
  // P = mat_coeff * [dcm_ini^T; \dot{dcm}_ini^T; dcm_end^T; \dot{dcm}_end]
  // where mat_coeff \in R^{4x4} and ^T denotes a vector transpose transpose.
  // Ts is the duration in seconds
  Eigen::MatrixXd polynomialMatrix(const double Ts,
                                   const Eigen::Vector3d & dcm_ini, const Eigen::Vector3d & dcm_vel_ini,
                                   const Eigen::Vector3d & dcm_end, const Eigen::Vector3d & dcm_vel_end);


  int clampINT(int input, int low_bound, int upper_bound);
  double clampDOUBLE(double input, double low_bound, double upper_bound);

};

#endif