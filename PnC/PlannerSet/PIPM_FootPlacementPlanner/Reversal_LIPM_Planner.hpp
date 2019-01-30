#pragma once
#include <vector>

#include <Eigen/Dense>

#include <PnC/FootStepPlanner.hpp>

class ParamReversalPL{
public:
  double swing_time;
  Eigen::Vector2d  des_loc;
  Eigen::Vector3d stance_foot_loc;
  bool b_positive_sidestep;
  double yaw_angle;
};

class OutputReversalPL{
public:
  double time_modification;
  double switching_state[4];
};

class Reversal_LIPM_Planner: public FootStepPlanner {
public:
  Reversal_LIPM_Planner();
  virtual ~Reversal_LIPM_Planner();

  virtual void PlannerInitialization(const YAML::Node & node);

  virtual void getNextFootLocation(const Eigen::Vector3d & com_pos,
                                   const Eigen::Vector3d & com_vel,
                                   Eigen::Vector3d & target_loc,
                                   const void* additional_input = NULL,
                                   void* additional_output = NULL);

  // Set Functions
  void setOmega(double com_height){
    b_set_omega_ = true;
    omega_ = sqrt(9.81/com_height);
  }
  void CheckEigenValues(double swing_time);

protected:
  // current com state: (x, y, xdot, ydot) : 4
  // switching com state: (x, y, xdot, ydot) : 4
  // target foot: (x, y) : 2
  // swing time: t : 1
  Eigen::VectorXd planner_save_data_;

  std::vector<double> t_prime_;
  std::vector<double> kappa_;
  std::vector<double> x_step_length_limit_;
  std::vector<double> y_step_length_limit_;
  std::vector<double> com_vel_limit_;
  Eigen::MatrixXd R_w_t_;

  double omega_;
  bool b_set_omega_;

  void _computeSwitchingState(double swing_time,
                              const Eigen::Vector3d& com_pos,
                              const Eigen::Vector3d& com_vel,
                              const Eigen::Vector3d& stance_foot_loc,
                              std::vector< Eigen::Vector2d > & switching_state);
  void _StepLengthCheck( Eigen::Vector3d & target_loc,
                        const std::vector< Eigen::Vector2d > & switching_state);
  void _StepLengthCheck( Eigen::Vector3d & target_loc,
                         bool b_positive_sidestep,
                         const Eigen::Vector3d & stance_foot);
  void _StepLengthCheckConsideringRotation( Eigen::Vector3d & target_loc,
                                            bool b_positive_sidestep,
                                            const Eigen::Vector3d & stance_foot);
  void _UpdateRotation( double yaw_angle );
  int _check_switch_velocity(const std::vector< Eigen::Vector2d > & switch_state);

};
