#pragma once

#include <PnC/DracoPnC/DracoDefinition.hpp>
#include <PnC/DracoPnC/DracoInterface.hpp>
#include <PnC/DracoPnC/DracoStateProvider.hpp>
#include <PnC/DracoPnC/DracoTaskAndForceContainer/DracoTaskAndForceContainer.hpp>
#include <PnC/WBC/JointIntegrator.hpp>
#include <PnC/WBC/WBC.hpp>
#include <PnC/WBC/WBLC/KinWBC.hpp>
#include <PnC/WBC/WBLC/WBLC.hpp>
#include <PnC/WBC/WBLC/AWBC.hpp>
#include <Utils/Math/pseudo_inverse.hpp>
// #include <PnC/WBC/WBDC/WBDC.hpp>

class filter;

class DracoWBCController {
 public:
  DracoWBCController(DracoTaskAndForceContainer* _taf_container,
                      RobotSystem* _robot);
  virtual ~DracoWBCController();

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
  // Controller Objects : KinWBC and WBLC (No double integrations)
  // -------------------------------------------------------
  KinWBC* kin_wbc_;
  WBLC* wblc_;
  WBLC_ExtraData* wblc_data_;
  AWBC* awbc_;

  bool b_enable_torque_limits_;

  Eigen::VectorXd Fd_des_;
  Eigen::VectorXd tau_cmd_;
  Eigen::VectorXd qddot_cmd_;

  Eigen::VectorXd des_jacc_;
  Eigen::VectorXd des_jpos_;
  Eigen::VectorXd des_jvel_;

  Eigen::VectorXd q_prev_;
  Eigen::VectorXd dq_prev_;

  Eigen::MatrixXd A_prev_;
  Eigen::MatrixXd H_prev_;
  Eigen::Vector3d pos_com_prev_;
  Eigen::Vector3d vel_com_prev_;
  Eigen::MatrixXd J_com_prev_;
  Eigen::MatrixXd AM_prev_;

  Eigen::MatrixXd H_;
  Eigen::Vector3d pos_com_;
  Eigen::Vector3d vel_com_;
  Eigen::MatrixXd J_com_;
  Eigen::MatrixXd AM_;

  Eigen::VectorXd hat_f_;

  // -------------------------------------------------------
  // Parameters
  // -------------------------------------------------------
  // WBC Controller parameters
  double lambda_xddot_;          // weight for the tasks
  double lambda_qddot_;          // weight for the qddot
  double lambda_rf_;             // weight for the reaction force
  int dim_contact_;

  double total_mass_;

  Eigen::MatrixXd A_;
  Eigen::MatrixXd Ainv_;
  Eigen::MatrixXd A_rotor_;
  Eigen::MatrixXd Ainv_rotor_;
  Eigen::MatrixXd grav_;
  Eigen::MatrixXd coriolis_;
  std::vector<Task*> task_list_;
  std::vector<ContactSpec*> contact_list_;

  Eigen::VectorXd Kp_;
  Eigen::VectorXd Kd_; 

  Eigen::VectorXd kp_;
  Eigen::VectorXd kd_;

  filter* x_force_ext_;
  filter* y_force_ext_;
  filter* z_force_ext_;

  filter* x_tau_ext_;
  filter* y_tau_ext_;
  filter* z_tau_ext_;
};
