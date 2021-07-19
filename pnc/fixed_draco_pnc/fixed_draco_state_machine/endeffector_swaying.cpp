#include "pnc/draco_pnc/draco_state_machine/endeffector_swaying.hpp"

EndEffectorSwaying::EndEffectorSwaying(const StateIdentifier _state_identifier,
                                       DracoControlArchitecture *_ctrl_arch,
                                       int _leg_side, RobotSystem *_robot)
    : StateMachine(_state_identifier, _robot) {

  util::PrettyConstructor(2, "EndEffectorSwaying");

  ctrl_arch_ = _ctrl_arch;
  leg_side_ = _leg_side;
  sp_ = DracoStateProvider::getStateProvider();
}

EndEffectorSwaying::~EndEffectorSwaying() {}

void EndEffectorSwaying::firstVisit() {
  if (leg_side_ == EndEffector::RFoot) {
    std::cout << "draco_states::kRFootSwaying" << std::endl;
  } else  {
    std::cout << "draco_states::kLFootSwaying" << std::endl;
  }

  ctrl_start_time_ = sp_->curr_time;

  ctrl_arch_->rf_ee_tm->initializeSwayingTrajectory(ctrl_start_time_, amp, freq);
  ctrl_arch_->lf_ee_tm->
}

void EndEffectorSwaying::oneStep() {
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;

  ctrl_arch_->ee_tm->updateDesired(sp_->curr_time);
}

void EndEffectorSwaying::lastVisit() {}

bool EndEffectorSwaying::endOfState() {return false;}

StateIdentifier EndEffectorSwaying::getNextState() {}
