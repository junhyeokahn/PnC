#include "PnC/draco_pnc/draco_state_machine/double_support_balance.hpp"

Initialize::Initialize(const StateIdentifier _state_identifier,
                       DracoControlArchitecture *_ctrl_arch,
                       RobotSystem *_robot)
    : StateMachine(_state_identifier, _robot) {

  myUtils::pretty_constructor(2, "SM: Initialize");

  ctrl_arch_ = _ctrl_arch;
  sp_ = DracoStateProvider::getStateProvider();
  end_time = 5.;

  target_jpos = Eigen::VectorXd::Zero(robot_->n_a);
  initial_jpos_ = Eigen::VectorXd::Zero(robot_->n_a);
}

Initialize::~Initialize() {}

void Initialize::firstVisit() {
  std::cout << "draco_states::kInitialize" << std::endl;

  ctrl_start_time_ = sp_->curr_time;
}

void Initialize::oneStep() {
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;

  // smooth changing joint position commands
  for (int i = 0; i < root_->n_a; ++i) {
  }
}

void Initialize::lastVisit() {}

bool Initialize::endOfState() {
  if (state_machine_time_ >= end_time) {
    return true;
  } else {
    return false;
  }
}

StateIdentifier Initialize::getNextState() { return draco_states::kStand; }
