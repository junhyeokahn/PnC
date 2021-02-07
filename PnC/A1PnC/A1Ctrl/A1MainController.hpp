#pragma once

#include <PnC/A1PnC/A1Definition.hpp>
#include <PnC/A1PnC/A1Interface.hpp>
#include <PnC/A1PnC/A1StateProvider.hpp>
#include <PnC/A1PnC/A1TaskAndForceContainer/A1TaskAndForceContainer.hpp>
#include <PnC/WBC/JointIntegrator.hpp>
#include <PnC/WBC/WBIC.hpp>
#include <PnC/WBC/KinWBC.hpp>

class A1MainController {
 public:
  A1MainController(A1TaskAndForceContainer* _taf_container,
                         RobotSystem* _robot);
  virtual ~A1MainController();

  void getCommand(void* _cmd);
  virtual void ctrlInitialization(const YAML::Node& node);

 protected:
  RobotSystem* robot_;
  A1TaskAndForceContainer* taf_container_;
  A1StateProvider* sp_;

  //  Processing Step for first visit
  void firstVisit();
  bool b_first_visit_;

  // Redefine PreProcessing Command
  void _PreProcessing_Command();
  // -------------------------------------------------------
  // Controller Objects
  // -------------------------------------------------------
  WBC* wbc_;
  IK* ik_;
  Eigen::VectorXd Fd_des_;
  Eigen::VectorXd tau_cmd_;
  Eigen::VectorXd qddot_cmd_;

  JointIntegrator* joint_integrator_;
  Eigen::VectorXd des_jacc_;
  Eigen::VectorXd des_jpos_;
  Eigen::VectorXd des_jvel_;

  // -------------------------------------------------------
  // Parameters
  // -------------------------------------------------------
  // IHWBC Controller parameters
  double wbc_dt_;                // Joint integration time
  double w_contact_weight_;      // contact weights hierarchy (relative to task
                                 // weights)
  double lambda_qddot_;          // Joint acceleration variable normalization
  double lambda_Fr_;             //  Reaction Force normalization
  bool b_enable_torque_limits_;  // Enable IHWBC torque limits

  // Joint Integrator parameters
  double vel_freq_cutoff_;  // Hz
  double pos_freq_cutoff_;  // Hz
  double max_pos_error_;  // radians. After position integrator, deviation from
                          // current position

  Eigen::MatrixXd A_;
  Eigen::MatrixXd Ainv_;
  Eigen::MatrixXd grav_;
  Eigen::MatrixXd coriolis_;
  std::vector<Task*> task_list_;
  std::vector<ContactSpec*> contact_list_;
};
