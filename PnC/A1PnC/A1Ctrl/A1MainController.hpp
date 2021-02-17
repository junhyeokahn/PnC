#pragma once

#include <PnC/A1PnC/A1Definition.hpp>
#include <PnC/A1PnC/A1Interface.hpp>
#include <PnC/A1PnC/A1StateProvider.hpp>
#include <PnC/A1PnC/A1TaskAndForceContainer/A1TaskAndForceContainer.hpp>
#include <PnC/WBC/JointIntegrator.hpp>
// #include <PnC/WBC/WBC.hpp>
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
  WBIC* wbic_;
  KinWBC* kin_wbc_;
  WBIC_ExtraData* wbic_data_;

  Eigen::VectorXd Fd_des_;
  Eigen::VectorXd tau_cmd_;
  Eigen::VectorXd qddot_cmd_;

  Eigen::VectorXd des_jacc_;
  Eigen::VectorXd des_jpos_;
  Eigen::VectorXd des_jvel_;


  Eigen::MatrixXd A_;
  Eigen::MatrixXd Ainv_;
  Eigen::MatrixXd grav_;
  Eigen::MatrixXd coriolis_;

  std::vector<Task*> task_list_;
  std::vector<ContactSpec*> contact_list_;

  std::vector<double> _Kp_joint, _Kd_joint;
};
