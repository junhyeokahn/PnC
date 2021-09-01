#include "pnc/draco_pnc/draco_state_machine/foot_swing.hpp"

FootSwing::FootSwing(const StateIdentifier _state_identifier,
                                       DracoControlArchitecture *_ctrl_arch,
                                       int _leg_side, RobotSystem *_robot)
    : StateMachine(_state_identifier, _robot) {

  util::PrettyConstructor(2, "FootSwing");

  ctrl_arch_ = _ctrl_arch;
  leg_side_ = _leg_side;
  sp_ = DracoStateProvider::getStateProvider();

  swing_duration_ = 3.;
  des_foot_x_increment_ = 0.1;
  des_foot_y_increment_ = 0.;
  des_foot_ori_increment_ = 0.;
}

FootSwing::~FootSwing() {}

void FootSwing::firstVisit() {
  if (leg_side_ == EndEffector::RFoot) {
    std::cout << "draco_states::kRFootSwingStatic" << std::endl;
  } else {
    std::cout << "draco_states::kLFootSwingStatic" << std::endl;
  }

  ctrl_start_time_ = sp_->curr_time;

  if (leg_side_ == EndEffector::RFoot) {
      Eigen::Vector3d local_des_foot_pos(des_foot_x_increment_,des_foot_y_increment_,0.);
      Eigen::Vector3d des_foot_pos = robot_->get_link_iso("r_foot_contact").translation() 
          + robot_->get_link_iso("r_foot_contact").linear() * local_des_foot_pos; 
      //TEST
      std::cout << "================des_foot_pos==============" << std::endl;
      std::cout << des_foot_pos << std::endl;
      //TEST
      Eigen::Matrix3d local_des_foot_ori;
      local_des_foot_ori.col(0) << cos(M_PI/180*des_foot_ori_increment_), -sin(M_PI/180*des_foot_ori_increment_), 0;
      local_des_foot_ori.col(1) << sin(M_PI/180*des_foot_ori_increment_), cos(M_PI/180*des_foot_ori_increment_), 0;
      local_des_foot_ori.col(2) << 0, 0, 1;
      Eigen::Quaternion<double> des_foot_ori(robot_->get_link_iso("r_foot_contact").linear()*local_des_foot_ori);

      Footstep landing_foot(des_foot_pos, des_foot_ori, EndEffector::RFoot);


      ctrl_arch_->rfoot_tm->InitializeSwingTrajectory(
              sp_->curr_time, swing_duration_,landing_foot);


  } else if (leg_side_ == EndEffector::LFoot) {
      Eigen::Vector3d local_des_foot_pos(des_foot_x_increment_,des_foot_y_increment_,0.);
      Eigen::Vector3d des_foot_pos = robot_->get_link_iso("l_foot_contat").translation() 
          + robot_->get_link_iso("l_foot_contact").linear() * local_des_foot_pos; 
      Eigen::Matrix3d local_des_foot_ori;
      local_des_foot_ori.col(0) << cos(M_PI/180*des_foot_ori_increment_), -sin(M_PI/180*des_foot_ori_increment_), 0;
      local_des_foot_ori.col(1) << sin(M_PI/180*des_foot_ori_increment_), cos(M_PI/180*des_foot_ori_increment_), 0;
      local_des_foot_ori.col(2) << 0, 0, 1;
      Eigen::Quaternion<double> des_foot_ori(robot_->get_link_iso("l_foot_contact").linear()*local_des_foot_ori);

      Footstep landing_foot(des_foot_pos, des_foot_ori, EndEffector::RFoot);

      ctrl_arch_->lfoot_tm->InitializeSwingTrajectory(
              sp_->curr_time, swing_duration_,landing_foot);

  } else {
      assert(false);
  }
}

void FootSwing::oneStep() {
    std::cout << "111111" << std::endl;
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;

  // Update Foot Task
  if (leg_side_ == EndEffector::LFoot) {
    ctrl_arch_->lfoot_tm->UpdateDesired(sp_->curr_time);
    ctrl_arch_->rfoot_tm->UpdateZeroAccCmd();
  } else {
    ctrl_arch_->rfoot_tm->UpdateDesired(sp_->curr_time);
    ctrl_arch_->lfoot_tm->UpdateZeroAccCmd();
  }

  // Update floating base task
}

void FootSwing::lastVisit() {
}

bool FootSwing::endOfState() {

  // if (state_machine_time_ >= swing_duration_) {
  // return true;
  //} else {
  // if (state_machine_time_ >= 0.5 * swing_duration_) {
  // if (leg_side_ == EndEffector::LFoot) {
  // if (sp_->b_lf_contact) {
  // printf("Early left foot contact at %f/%f\n", state_machine_time_,
  // swing_duration_);
  // return true;
  //}
  //} else {
  // if (sp_->b_rf_contact) {
  // printf("Early right foot contact at %f/%f\n", state_machine_time_,
  // swing_duration_);
  // return true;
  //}
  //}
  //}
  // return false;
  //}

  if (state_machine_time_ >= swing_duration_) {
    return true;
  } else {
    return false;
  }
}

StateIdentifier FootSwing::getNextState() {
    if (leg_side_ == EndEffector::LFoot) {
        return draco_states::kLFootLanding;
    } else {
        return draco_states::kRFootLanding;
    }
}
