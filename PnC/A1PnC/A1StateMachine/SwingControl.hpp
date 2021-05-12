#pragma once

#include <Utils/Math/BSplineBasic.h>
#include <PnC/A1PnC/A1StateProvider.hpp>
#include <PnC/StateMachine.hpp>

class A1ControlArchitecture;
class A1TaskAndForceContainer;

class SwingControl : public StateMachine {
 public:
  SwingControl(const StateIdentifier state_identifier_in, const int _leg_side,
               A1ControlArchitecture* _ctrl_arch, RobotSystem* _robot);
  ~SwingControl();

  void oneStep();
  void firstVisit();
  void lastVisit();
  bool endOfState();
  void initialization(const YAML::Node& node);
  StateIdentifier getNextState();

 protected:
  A1StateProvider* sp_;
  A1ControlArchitecture* ctrl_arch_;
  A1TaskAndForceContainer* taf_container_;

  int leg_side_;

  double ctrl_start_time_;
  double swing_duration_;
  double end_time_;

  double swing_time_percent_early_contact_check_;
  double early_contact_force_threshold_;

  void _taskUpdate();

  void footstepPlanner();
  void sangbaeFootstepPlanner();
  void donghyunFootstepPlanner();

  // Footstep Planning Variables
  Eigen::VectorXd p_sh, p_sym, p_cent;
  Eigen::VectorXd front_foot_end_pos;
  Eigen::VectorXd rear_foot_end_pos;
  Eigen::MatrixXd rot_;
  double yaw;
};
