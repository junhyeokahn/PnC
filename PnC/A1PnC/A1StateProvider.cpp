#include <PnC/A1PnC/A1Definition.hpp>
#include <PnC/A1PnC/A1StateProvider.hpp>
#include <PnC/RobotSystem/RobotSystem.hpp>
#include <Utils/IO/DataManager.hpp>

A1StateProvider* A1StateProvider::getStateProvider(RobotSystem* _robot) {
  static A1StateProvider state_provider_(_robot);
  return &state_provider_;
}

A1StateProvider::A1StateProvider(RobotSystem* _robot) {
  myUtils::pretty_constructor(1, "State Provider");

  robot_ = _robot;
  curr_time = 0.;
  planning_id = 0;
  prev_trq_cmd = Eigen::VectorXd::Zero(12);

  rotor_inertia = Eigen::VectorXd::Zero(12);
  q = Eigen::VectorXd::Zero(18);
  qdot = Eigen::VectorXd::Zero(18);
  q_task_des = Eigen::VectorXd::Zero(12);
  qdot_task_des = Eigen::VectorXd::Zero(12);
  q_task = Eigen::VectorXd::Zero(12);
  qdot_task = Eigen::VectorXd::Zero(12);
  b_frfoot_contact = 0;
  b_flfoot_contact = 0;
  b_rrfoot_contact = 0;
  b_rlfoot_contact = 0;
  qddot_cmd = Eigen::VectorXd::Zero(18);

  frfoot_pos_des.setZero();
  flfoot_pos_des.setZero();
  rrfoot_pos_des.setZero();
  rlfoot_pos_des.setZero();
  frfoot_vel_des.setZero();
  flfoot_vel_des.setZero();
  rrfoot_vel_des.setZero();
  rlfoot_vel_des.setZero();

  frfoot_pos.setZero();
  flfoot_pos.setZero();
  rrfoot_pos.setZero();
  rlfoot_pos.setZero();
  frfoot_vel.setZero();
  flfoot_vel.setZero();
  rrfoot_vel.setZero();
  rlfoot_vel.setZero();

  com_pos.setZero();
  com_vel.setZero();
  com_pos_des.setZero();
  com_vel_des.setZero();
  // est_com_vel.setZero();

  /*fr_rf = 0; // Eigen::VectorXd::Zero(6);
  fl_rf = 0; // Eigen::VectorXd::Zero(6);
  rr_rf = 0; // Eigen::VectorXd::Zero(6);
  rl_rf = 0; // Eigen::VectorXd::Zero(6);
  fr_rf_des = 0; // Eigen::VectorXd::Zero(6);
  fl_rf_des = 0; // Eigen::VectorXd::Zero(6);
  rr_rf_des = 0; // Eigen::VectorXd::Zero(6);
  rl_rf_des = 0; // Eigen::VectorXd::Zero(6);*/

  DataManager* data_manager = DataManager::GetDataManager();

  // ---------------------------------------------------------------------------
  // Robot States
  // ---------------------------------------------------------------------------
  data_manager->RegisterData(&q, VECT, "config", robot_->getNumDofs());
  data_manager->RegisterData(&qdot, VECT, "qdot", robot_->getNumDofs());

  // ---------------------------------------------------------------------------
  // Contact
  // ---------------------------------------------------------------------------
  data_manager->RegisterData(&b_frfoot_contact, INT, "frfoot_contact", 1);
  data_manager->RegisterData(&b_flfoot_contact, INT, "flfoot_contact", 1);
  /*data_manager->RegisterData(&fr_rf, INT, "fr_rf", 1);
  data_manager->RegisterData(&fl_rf, INT, "fl_rf", 1);
  data_manager->RegisterData(&fr_rf_des, INT, "fr_rf_des", 1);
  data_manager->RegisterData(&fl_rf_des, INT, "fl_rf_des", 1);
  data_manager->RegisterData(&b_rrfoot_contact, INT, "rrfoot_contact", 1);
  data_manager->RegisterData(&b_rlfoot_contact, INT, "rlfoot_contact", 1);
  data_manager->RegisterData(&rr_rf, INT, "rr_rf", 1);
  data_manager->RegisterData(&rl_rf, INT, "rl_rf", 1);
  data_manager->RegisterData(&rr_rf_des, INT, "rr_rf_des", 1);
  data_manager->RegisterData(&rl_rf_des, INT, "rl_rf_des", 1);*/

  // ---------------------------------------------------------------------------
  // WBC
  // ---------------------------------------------------------------------------
  data_manager->RegisterData(&qddot_cmd, VECT, "qddot_des", 16);
  data_manager->RegisterData(&w_frfoot_pos, DOUBLE, "w_frfoot_pos", 1);
  data_manager->RegisterData(&w_flfoot_pos, DOUBLE, "w_flfoot_pos", 1);
  data_manager->RegisterData(&w_rrfoot_pos, DOUBLE, "w_rrfoot_pos", 1);
  data_manager->RegisterData(&w_rlfoot_pos, DOUBLE, "w_rlfoot_pos", 1);
  data_manager->RegisterData(&w_com, DOUBLE, "w_com", 1);
  data_manager->RegisterData(&w_base_ori, DOUBLE, "w_base_ori", 1);
  data_manager->RegisterData(&w_frfoot_fr, DOUBLE, "w_frfoot_fr", 1);
  data_manager->RegisterData(&w_flfoot_fr, DOUBLE, "w_flfoot_fr", 1);
  data_manager->RegisterData(&w_rrfoot_fr, DOUBLE, "w_rrfoot_fr", 1);
  data_manager->RegisterData(&w_rlfoot_fr, DOUBLE, "w_rlfoot_fr", 1);

  // ---------------------------------------------------------------------------
  // Foot
  // ---------------------------------------------------------------------------
    // NOT YET USING
  /*data_manager->RegisterData(&frfoot_pos, VECT3, "frfoot_pos", 3);
  data_manager->RegisterData(&flfoot_pos, VECT3, "flfoot_pos", 3);
  data_manager->RegisterData(&frfoot_vel, VECT3, "frfoot_vel", 3);
  data_manager->RegisterData(&flfoot_vel, VECT3, "flfoot_vel", 3);

  data_manager->RegisterData(&rrfoot_pos, VECT3, "rrfoot_pos", 3);
  data_manager->RegisterData(&rlfoot_pos, VECT3, "rlfoot_pos", 3);
  data_manager->RegisterData(&rrfoot_vel, VECT3, "rrfoot_vel", 3);
  data_manager->RegisterData(&rlfoot_vel, VECT3, "rlfoot_vel", 3);

  data_manager->RegisterData(&frfoot_pos_des, VECT3, "frfoot_pos_des", 3);
  data_manager->RegisterData(&flfoot_pos_des, VECT3, "flfoot_pos_des", 3);
  data_manager->RegisterData(&frfoot_vel_des, VECT3, "frfoot_vel_des", 3);
  data_manager->RegisterData(&flfoot_vel_des, VECT3, "flfoot_vel_des", 3);

  data_manager->RegisterData(&rrfoot_pos_des, VECT3, "rrfoot_pos_des", 3);
  data_manager->RegisterData(&rlfoot_pos_des, VECT3, "rlfoot_pos_des", 3);
  data_manager->RegisterData(&rrfoot_vel_des, VECT3, "rrfoot_vel_des", 3);
  data_manager->RegisterData(&rlfoot_vel_des, VECT3, "rlfoot_vel_des", 3);*/

  // ---------------------------------------------------------------------------
  // COM
  // ---------------------------------------------------------------------------
  data_manager->RegisterData(&com_pos, VECT3, "com_pos", 3);
  data_manager->RegisterData(&com_vel, VECT3, "com_vel", 3);
  //data_manager->RegisterData(&est_com_vel, VECT3, "est_com_vel", 3);
  data_manager->RegisterData(&com_pos_des, VECT3, "com_pos_des", 3);
  data_manager->RegisterData(&com_vel_des, VECT3, "com_vel_des", 3);
  data_manager->RegisterData(&base_quat, QUATERNION, "base_quat", 4);
  data_manager->RegisterData(&base_ang_vel, VECT3, "base_ang_vel", 3);
  data_manager->RegisterData(&base_quat_des, QUATERNION, "base_quat_des", 4);
  data_manager->RegisterData(&base_ang_vel_des, VECT3, "base_ang_vel_des", 3);

  // ---------------------------------------------------------------------------
  // Joint
  // ---------------------------------------------------------------------------
  /*data_manager->RegisterData(&q_task_des, VECT, "q_task_des", 12);
  data_manager->RegisterData(&qdot_task_des, VECT, "qdot_task_des", 12);
  data_manager->RegisterData(&q_task, VECT, "q_task", 12);
  data_manager->RegisterData(&qdot_task, VECT, "qdot_task", 12);*/
}

void A1StateProvider::saveCurrentData() {
  frfoot_pos =
      robot_->getBodyNodeIsometry(A1BodyNode::FR_foot).translation();
  flfoot_pos =
      robot_->getBodyNodeIsometry(A1BodyNode::FL_foot).translation();
  rrfoot_pos =
      robot_->getBodyNodeIsometry(A1BodyNode::RR_foot).translation();
  rlfoot_pos =
      robot_->getBodyNodeIsometry(A1BodyNode::RL_foot).translation();

  frfoot_vel =
      robot_->getBodyNodeSpatialVelocity(A1BodyNode::FR_foot).tail(3);
  flfoot_vel =
      robot_->getBodyNodeSpatialVelocity(A1BodyNode::FL_foot).tail(3);
  rrfoot_vel =
      robot_->getBodyNodeSpatialVelocity(A1BodyNode::RR_foot).tail(3);
  rlfoot_vel =
      robot_->getBodyNodeSpatialVelocity(A1BodyNode::RL_foot).tail(3);

  base_quat = Eigen::Quaternion<double>(
      robot_->getBodyNodeIsometry(A1BodyNode::trunk).linear());
  base_ang_vel =
      robot_->getBodyNodeCoMSpatialVelocity(A1BodyNode::trunk).head(3);
}
