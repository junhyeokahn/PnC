#pragma once

#include <PnC/LaikagoPnC/LaikagoDefinition.hpp>
#include <PnC/LaikagoPnC/LaikagoInterface.hpp>
#include <PnC/LaikagoPnC/LaikagoStateProvider.hpp>
#include <PnC/LaikagoPnC/LaikagoTaskAndForceContainer/LaikagoTaskAndForceContainer.hpp>
#include <PnC/WBC/JointIntegrator.hpp>
#include <PnC/WBC/WBC.hpp>

class LaikagoMainController {
 public:
  LaikagoMainController(LaikagoTaskAndForceContainer* _taf_container,
                      RobotSystem* _robot);
  virtual ~LaikagoMainController();

  void getCommand(void* _cmd);
  virtual void ctrlInitialization(const YAML::Node& node);
  void initializeJointIntegrator();

 protected:
  RobotSystem* robot_;
  LaikagoTaskAndForceContainer* taf_container_;
  LaikagoStateProvider* sp_;

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
  double max_pos_error_;    // radians. After position integrator, deviation
                            // from current position

  Eigen::MatrixXd A_;
  Eigen::MatrixXd Ainv_;
  Eigen::MatrixXd A_rotor_;
  Eigen::MatrixXd Ainv_rotor_;
  Eigen::MatrixXd grav_;
  Eigen::MatrixXd coriolis_;
  std::vector<Task*> task_list_;
  std::vector<ContactSpec*> contact_list_;
};
