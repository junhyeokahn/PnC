#pragma once

#include <PnC/RobotSystem/RobotSystem.hpp>
#include <PnC/WBC/Task.hpp>
#include <PnC/WBC/ContactSpec.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <memory>

typedef int StateIdentifier;

class StateMachine{
public:
  StateMachine(const StateIdentifier state_identifier_in, RobotSystem* _robot) {
      robot_ = _robot;
      state_machine_time_ = 0.;
      state_identity_ = state_identifier_in;
  }
  virtual ~StateMachine(){}

  virtual void oneStep(void* command) = 0;
  virtual void firstVisit() = 0;
  virtual void lastVisit() = 0;
  virtual bool endOfState() = 0;
  virtual void Initialization(const YAML::Node& node) = 0;
  virtual StateIdentifier getNextState() = 0;

  StateIdentifier getStateIdentity(){
    return state_identity_;
  }

protected:
  void _PreProcessing_Command(){
      A_ = robot_->getMassMatrix();
      Ainv_ = robot_->getInvMassMatrix();
      grav_ = robot_->getGravity();
      coriolis_ = robot_->getCoriolis();
  }

  void _PostProcessing_Command(){
  }

  RobotSystem* robot_;
  StateIdentifier state_identity_; // unique integer of this state

  Eigen::MatrixXd A_;
  Eigen::MatrixXd Ainv_;
  Eigen::VectorXd grav_;
  Eigen::VectorXd coriolis_;

  double state_machine_time_;
};
