#pragma once

#include <PnC/DracoPnC/DracoDefinition.hpp>
#include <PnC/DracoPnC/DracoInterface.hpp>
#include <PnC/DracoPnC/DracoStateProvider.hpp>
#include <PnC/DracoPnC/DracoTaskAndForceContainer/DracoTaskAndForceContainer.hpp>
#include <PnC/WBC/JointIntegrator.hpp>
#include <PnC/WBC/WBC.hpp>

class DracoMainController {
 public:
  DracoMainController(DracoTaskAndForceContainer* _taf_container,
                      RobotSystem* _robot);
  virtual ~DracoMainController();

  void getCommand(void* _cmd);
  virtual void ctrlInitialization(const YAML::Node& node);

 protected:
  RobotSystem* robot_;
  DracoTaskAndForceContainer* taf_container_;
  DracoStateProvider* sp_;

  //  Processing Step for first visit
  void firstVisit();
  bool b_first_visit_;

  // Redefine PreProcessing Command
  void _PreProcessing_Command();
  // -------------------------------------------------------
  // Controller Objects
  // -------------------------------------------------------
  WBC* wbc_;
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
