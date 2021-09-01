#include "pnc/draco_pnc/draco_state_machine/initialize.hpp"

Initialize::Initialize(const StateIdentifier _state_identifier,
                       DracoControlArchitecture *_ctrl_arch,
                       RobotSystem *_robot)
    : StateMachine(_state_identifier, _robot) {

  util::PrettyConstructor(2, "Initialize");

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

  initial_jpos_ = robot_->joint_positions;
}

void Initialize::oneStep() {
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;

  // smooth changing joint position commands
  Eigen::VectorXd des_jpos = Eigen::VectorXd::Zero(robot_->n_a);
  Eigen::VectorXd des_jvel = Eigen::VectorXd::Zero(robot_->n_a);
  Eigen::VectorXd des_jacc = Eigen::VectorXd::Zero(robot_->n_a);
  for (int i = 0; i < robot_->n_a; ++i) {
    des_jpos[i] = util::SmoothPos(initial_jpos_[i], target_jpos[i], end_time,
                                  state_machine_time_);
    des_jvel[i] = util::SmoothVel(initial_jpos_[i], target_jpos[i], end_time,
                                  state_machine_time_);
    des_jacc[i] = util::SmoothAcc(initial_jpos_[i], target_jpos[i], end_time,
                                  state_machine_time_);
  }
  ctrl_arch_->tci_container->joint_task->update_desired(des_jpos, des_jvel,
                                                        des_jacc);
}

void Initialize::lastVisit() {}

bool Initialize::endOfState() {
  if (state_machine_time_ >= end_time) {
    return true;
  } else {
    return false;
  }
  // return false;
}

StateIdentifier Initialize::getNextState() { return draco_states::kStand; }
