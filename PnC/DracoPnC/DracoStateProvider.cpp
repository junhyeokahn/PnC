#include <PnC/DracoPnC/DracoDefinition.hpp>
#include <PnC/DracoPnC/DracoStateProvider.hpp>
#include <PnC/RobotSystem/RobotSystem.hpp>
#include <Utils/IO/DataManager.hpp>

DracoStateProvider* DracoStateProvider::getStateProvider(RobotSystem* _robot) {
  static DracoStateProvider state_provider_(_robot);
  return &state_provider_;
}

DracoStateProvider::DracoStateProvider(RobotSystem* _robot) {
  myUtils::pretty_constructor(1, "State Provider");

  robot_ = _robot;
  stance_foot = DracoBodyNode::lFootCenter;
  prev_stance_foot = stance_foot;
  curr_time = 0.;
  planning_id = 0;
  prev_trq_cmd = Eigen::VectorXd::Zero(10);

  rotor_inertia = Eigen::VectorXd::Zero(10);
  q = Eigen::VectorXd::Zero(robot_->getNumDofs());
  qdot = Eigen::VectorXd::Zero(robot_->getNumDofs());
  q_task_des = Eigen::VectorXd::Zero(10);
  qdot_task_des = Eigen::VectorXd::Zero(10);
  q_task = Eigen::VectorXd::Zero(10);
  qdot_task = Eigen::VectorXd::Zero(10);
  b_rfoot_contact = 0;
  b_lfoot_contact = 0;
  qddot_cmd = Eigen::VectorXd::Zero(16);

  rfoot_center_pos_des.setZero();
  lfoot_center_pos_des.setZero();
  rfoot_center_vel_des.setZero();
  lfoot_center_vel_des.setZero();
  rfoot_center_quat_des = Eigen::Quaternion<double>::Identity();
  lfoot_center_quat_des = Eigen::Quaternion<double>::Identity();
  rfoot_center_so3_des.setZero();
  lfoot_center_so3_des.setZero();

  rfoot_center_pos.setZero();
  lfoot_center_pos.setZero();
  rfoot_center_vel.setZero();
  lfoot_center_vel.setZero();
  rfoot_center_quat = Eigen::Quaternion<double>::Identity();
  lfoot_center_quat = Eigen::Quaternion<double>::Identity();
  rfoot_center_so3.setZero();
  lfoot_center_so3.setZero();

  com_pos.setZero();
  com_vel.setZero();
  com_pos_des.setZero();
  com_vel_des.setZero();
  est_com_vel.setZero();

  dcm.setZero();
  dcm_des.setZero();
  prev_dcm.setZero();
  dcm_vel.setZero();
  dcm_vel_des.setZero();
  r_vrp.setZero();
  r_vrp_des.setZero();
  dcm_omega = 1.0;

  r_rf = Eigen::VectorXd::Zero(6);
  l_rf = Eigen::VectorXd::Zero(6);
  r_rf_des = Eigen::VectorXd::Zero(6);
  l_rf_des = Eigen::VectorXd::Zero(6);

//   r_rf_front_des.setZero();
//   r_rf_back_des.setZero();
//   l_rf_front_des.setZero();
//   l_rf_back_des.setZero();

    r_front_rf  = Eigen::VectorXd::Zero(6);
    l_front_rf  = Eigen::VectorXd::Zero(6);
    r_back_rf  = Eigen::VectorXd::Zero(6);
    l_back_rf  = Eigen::VectorXd::Zero(6);

  DataManager* data_manager = DataManager::GetDataManager();

  // ---------------------------------------------------------------------------
  // Robot States
  // ---------------------------------------------------------------------------
  data_manager->RegisterData(&q, VECT, "config", robot_->getNumDofs());
  data_manager->RegisterData(&qdot, VECT, "qdot", robot_->getNumDofs());

  // ---------------------------------------------------------------------------
  // Contact
  // ---------------------------------------------------------------------------
  data_manager->RegisterData(&b_rfoot_contact, INT, "rfoot_contact", 1);
  data_manager->RegisterData(&b_lfoot_contact, INT, "lfoot_contact", 1);
  data_manager->RegisterData(&r_rf, VECT, "r_rf", 6);
  data_manager->RegisterData(&l_rf, VECT, "l_rf", 6);
  data_manager->RegisterData(&r_rf_des, VECT, "r_rf_des", 6);
  data_manager->RegisterData(&l_rf_des, VECT, "l_rf_des", 6);
  data_manager->RegisterData(&r_rf_front_des, VECT3, "r_rf_front_des", 3);
  data_manager->RegisterData(&r_rf_back_des, VECT3, "r_rf_back_des", 3);
  data_manager->RegisterData(&l_rf_front_des, VECT3, "l_rf_front_des", 3);
  data_manager->RegisterData(&l_rf_back_des, VECT3, "l_rf_back_des", 3);

  // ---------------------------------------------------------------------------
  // WBC
  // ---------------------------------------------------------------------------
  data_manager->RegisterData(&qddot_cmd, VECT, "qddot_des", 16);
  data_manager->RegisterData(&w_rfoot_pos, DOUBLE, "w_rfoot_pos", 1);
  data_manager->RegisterData(&w_rfoot_ori, DOUBLE, "w_rfoot_ori", 1);
  data_manager->RegisterData(&w_lfoot_pos, DOUBLE, "w_lfoot_pos", 1);
  data_manager->RegisterData(&w_lfoot_ori, DOUBLE, "w_lfoot_ori", 1);
  data_manager->RegisterData(&w_com, DOUBLE, "w_com", 1);
  data_manager->RegisterData(&w_base_ori, DOUBLE, "w_base_ori", 1);
  data_manager->RegisterData(&w_rf_rffront, DOUBLE, "w_rf_rffront", 1);
  data_manager->RegisterData(&w_rf_rfback, DOUBLE, "w_rf_rfback", 1);
  data_manager->RegisterData(&w_rf_lffront, DOUBLE, "w_rf_lffront", 1);
  data_manager->RegisterData(&w_rf_lfback, DOUBLE, "w_rf_lfback", 1);
  data_manager->RegisterData(&w_rfoot_fr, DOUBLE, "w_rfoot_fr", 1);
  data_manager->RegisterData(&w_lfoot_fr, DOUBLE, "w_lfoot_fr", 1);

  // ---------------------------------------------------------------------------
  // Foot
  // ---------------------------------------------------------------------------
  data_manager->RegisterData(&rfoot_center_pos, VECT3, "rfoot_pos", 3);
  data_manager->RegisterData(&lfoot_center_pos, VECT3, "lfoot_pos", 3);
  data_manager->RegisterData(&rfoot_center_vel, VECT3, "rfoot_vel", 3);
  data_manager->RegisterData(&lfoot_center_vel, VECT3, "lfoot_vel", 3);
  data_manager->RegisterData(&rfoot_center_so3, VECT3, "rfoot_so3", 3);
  data_manager->RegisterData(&lfoot_center_so3, VECT3, "lfoot_so3", 3);
  data_manager->RegisterData(&rfoot_center_quat, QUATERNION, "rfoot_quat", 4);
  data_manager->RegisterData(&lfoot_center_quat, QUATERNION, "lfoot_quat", 4);

  data_manager->RegisterData(&rfoot_center_pos_des, VECT3, "rfoot_pos_des", 3);
  data_manager->RegisterData(&lfoot_center_pos_des, VECT3, "lfoot_pos_des", 3);
  data_manager->RegisterData(&rfoot_center_vel_des, VECT3, "rfoot_vel_des", 3);
  data_manager->RegisterData(&lfoot_center_vel_des, VECT3, "lfoot_vel_des", 3);
  data_manager->RegisterData(&rfoot_center_so3_des, VECT3, "rfoot_so3_des", 3);
  data_manager->RegisterData(&lfoot_center_so3_des, VECT3, "lfoot_so3_des", 3);
  data_manager->RegisterData(&rfoot_center_quat_des, QUATERNION,
                             "rfoot_quat_des", 4);
  data_manager->RegisterData(&lfoot_center_quat_des, QUATERNION,
                             "lfoot_quat_des", 4);

  // ---------------------------------------------------------------------------
  // COM
  // ---------------------------------------------------------------------------
  data_manager->RegisterData(&com_pos, VECT3, "com_pos", 3);
  data_manager->RegisterData(&com_vel, VECT3, "com_vel", 3);
  data_manager->RegisterData(&est_com_vel, VECT3, "est_com_vel", 3);
  data_manager->RegisterData(&com_pos_des, VECT3, "com_pos_des", 3);
  data_manager->RegisterData(&com_vel_des, VECT3, "com_vel_des", 3);
  data_manager->RegisterData(&dcm_des, VECT3, "dcm_des", 3);
  data_manager->RegisterData(&dcm_vel_des, VECT3, "dcm_vel_des", 3);
  data_manager->RegisterData(&r_vrp_des, VECT3, "r_vrp_des", 3);
  data_manager->RegisterData(&r_vrp, VECT3, "r_vrp", 3);
  data_manager->RegisterData(&dcm, VECT3, "dcm", 3);
  data_manager->RegisterData(&dcm_vel, VECT3, "dcm_vel", 3);
  data_manager->RegisterData(&r_vrp, VECT3, "r_vrp", 3);
  data_manager->RegisterData(&base_quat, QUATERNION, "base_quat", 4);
  data_manager->RegisterData(&base_ang_vel, VECT3, "base_ang_vel", 3);
  data_manager->RegisterData(&base_quat_des, QUATERNION, "base_quat_des", 4);
  data_manager->RegisterData(&base_ang_vel_des, VECT3, "base_ang_vel_des", 3);

  // ---------------------------------------------------------------------------
  // Joint
  // ---------------------------------------------------------------------------
  data_manager->RegisterData(&q_task_des, VECT, "q_task_des", 10);
  data_manager->RegisterData(&qdot_task_des, VECT, "qdot_task_des", 10);
  data_manager->RegisterData(&q_task, VECT, "q_task", 10);
  data_manager->RegisterData(&qdot_task, VECT, "qdot_task", 10);
}

void DracoStateProvider::saveCurrentData() {
  rfoot_center_pos =
      robot_->getBodyNodeIsometry(DracoBodyNode::rFootCenter).translation();
  lfoot_center_pos =
      robot_->getBodyNodeIsometry(DracoBodyNode::lFootCenter).translation();
  rfoot_center_vel =
      robot_->getBodyNodeSpatialVelocity(DracoBodyNode::rFootCenter).tail(3);
  lfoot_center_vel =
      robot_->getBodyNodeSpatialVelocity(DracoBodyNode::lFootCenter).tail(3);

  rfoot_center_quat = Eigen::Quaternion<double>(
      robot_->getBodyNodeIsometry(DracoBodyNode::rFootCenter).linear());
  lfoot_center_quat = Eigen::Quaternion<double>(
      robot_->getBodyNodeIsometry(DracoBodyNode::lFootCenter).linear());
  rfoot_center_so3 =
      robot_->getBodyNodeSpatialVelocity(DracoBodyNode::rFootCenter).head(3);
  lfoot_center_so3 =
      robot_->getBodyNodeSpatialVelocity(DracoBodyNode::lFootCenter).head(3);

  base_quat = Eigen::Quaternion<double>(
      robot_->getBodyNodeIsometry(DracoBodyNode::Torso).linear());
  base_ang_vel =
      robot_->getBodyNodeCoMSpatialVelocity(DracoBodyNode::Torso).head(3);
}
