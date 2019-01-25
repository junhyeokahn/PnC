#pragma once

#include <Utils/pseudo_inverse.hpp>
#include <RobotSystem/RobotSystem.hpp>
#include <PnC/WBC/Task.hpp>
#include <PnC/WBC/ContactSpec.hpp>
#include <Utils/Utilities.hpp>

class Controller{
public:
  Controller(RobotSystem* _robot) {
      robot_ = _robot;
      state_machine_time_ = 0.;
  }
  virtual ~Controller(){}

  virtual void oneStep(void* command) = 0;
  virtual void firstVisit() = 0;
  virtual void lastVisit() = 0;
  virtual bool endOfPhase() = 0;
  virtual void ctrlInitialization(const YAML::Node& node) = 0;

protected:
  // TODO : replace it with utils
  void _DynConsistent_Inverse(const Eigen::MatrixXd & J, Eigen::MatrixXd & Jinv){
      Eigen::MatrixXd Jtmp(J * Ainv_ * J.transpose());
      Eigen::MatrixXd Jtmp_inv;
      myUtils::pseudoInverse(Jtmp, 0.0001, Jtmp_inv, 0);
      Jinv = Ainv_ * J.transpose() * Jtmp_inv;
  }

  void _PreProcessing_Command(){
      A_ = robot_->getMassMatrix();
      Ainv_ = robot_->getInvMassMatrix();
      grav_ = robot_->getGravity();
      coriolis_ = robot_->getCoriolis();

      task_list_.clear();
      contact_list_.clear();
  }

  void _PostProcessing_Command(){
      for(int i(0); i<task_list_.size(); ++i){ task_list_[i]->unsetTask(); }
      for(int i(0); i<contact_list_.size(); ++i){ contact_list_[i]->unsetContact(); }
  }

  RobotSystem* robot_;

  Eigen::MatrixXd A_;
  Eigen::MatrixXd Ainv_;
  Eigen::VectorXd grav_;
  Eigen::VectorXd coriolis_;

  std::vector<Task*> task_list_;
  std::vector<ContactSpec*> contact_list_;

  double state_machine_time_;
};
