#pragma once

#include <Utils/Math/BSplineBasic.h>
#include <PnC/LaikagoPnC/LaikagoStateProvider.hpp>
#include <PnC/StateMachine.hpp>

class LaikagoControlArchitecture;
class LaikagoTaskAndForceContainer;
class LaikagoMainController;

class ContactTransitionEnd : public StateMachine {
 public:
  ContactTransitionEnd(const StateIdentifier state_identifier_in,
                       const int _leg_side,
                       LaikagoControlArchitecture* _ctrl_arch,
                       RobotSystem* _robot);
  ~ContactTransitionEnd();

  void oneStep();
  void firstVisit();
  void lastVisit();
  bool endOfState();
  void initialization(const YAML::Node& node);
  StateIdentifier getNextState();

 protected:
  LaikagoStateProvider* sp_;
  LaikagoControlArchitecture* ctrl_arch_;
  LaikagoTaskAndForceContainer* taf_container_;

  int leg_side_;
  bool final_step_;

  double ctrl_start_time_;
  double end_time_;

  void _taskUpdate();
};
