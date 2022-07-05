#include "pnc/draco_pnc/draco_state_machine/double_support_stand.hpp"

DoubleSupportStand::DoubleSupportStand(const StateIdentifier _state_identifier,
                                       DracoControlArchitecture *_ctrl_arch,
                                       RobotSystem *_robot)
    : StateMachine(_state_identifier, _robot) {

  util::PrettyConstructor(2, "DoubleSupportStand");

  ctrl_arch_ = _ctrl_arch;
  sp_ = DracoStateProvider::getStateProvider();
}

DoubleSupportStand::~DoubleSupportStand() {}

void DoubleSupportStand::firstVisit() {
//  std::cout << "draco_states::kDoubleSupportStand" << std::endl;
  ctrl_start_time_ = sp_->curr_time;

  // Initialize CoM Trajectory
  Eigen::Isometry3d lfoot_iso = robot_->get_link_iso("l_foot_contact");
  Eigen::Isometry3d rfoot_iso = robot_->get_link_iso("r_foot_contact");

  Eigen::Vector3d target_com_pos =
      (lfoot_iso.translation() + rfoot_iso.translation()) / 2.;
  target_com_pos[2] = com_height_des;
  Eigen::Quaternion<double> foot_interpol_quat =
      Eigen::Quaternion<double>(lfoot_iso.linear())
          .slerp(0.5, Eigen::Quaternion<double>(rfoot_iso.linear()));

  Eigen::Matrix3d foot_interpol_SO3 = foot_interpol_quat.toRotationMatrix();
  Eigen::Vector3d ori_x(0, 0, 0), ori_y(0, 0, 0), ori_z(0, 0, 1);
  ori_y = foot_interpol_SO3.col(1);
  ori_x = ori_y.cross(ori_z);

  Eigen::Matrix3d target_base_ori_SO3 = Eigen::Matrix3d::Identity();
  target_base_ori_SO3.col(0) = ori_x;
  target_base_ori_SO3.col(1) = ori_y;
  target_base_ori_SO3.col(2) = ori_z;
  Eigen::Quaternion<double> target_base_ori =
      Eigen::Quaternion<double>(target_base_ori_SO3);

  std::cout << "rotation matrix: \n " << target_base_ori_SO3 << std::endl;
  std::cout << "quaternion before normalization: " << target_base_ori.coeffs().transpose() << std::endl;

  target_base_ori.normalize();
  sp_->nominal_base_quat = target_base_ori;
  std::cout << "quaternion after normalization: " << target_base_ori.coeffs().transpose() << std::endl;


  ori_y << lfoot_iso.linear().col(1);
  ori_x = ori_y.cross(ori_z);
  sp_->nominal_stance_foot_iso.linear().col(0) = ori_x;
  sp_->nominal_stance_foot_iso.linear().col(1) = ori_y;
  sp_->nominal_stance_foot_iso.linear().col(2) = ori_z;

  ctrl_arch_->floating_base_tm->InitializeInterpolationTrajectory(
      sp_->curr_time, end_time, target_com_pos, target_base_ori);

  std::cout << "[Nominal Base SO3]" << std::endl;
  std::cout << target_base_ori_SO3 << std::endl;
  std::cout << sp_->nominal_base_quat.toRotationMatrix() << std::endl;
  std::cout << "[Nominal Stance Foot SO3]" << std::endl;
  std::cout << sp_->nominal_stance_foot_iso.linear() << std::endl;

  // Initialize Reaction Force Ramp to Max
  ctrl_arch_->rfoot_fm->InitializeRampToMax(sp_->curr_time, rf_z_max_time);
  ctrl_arch_->lfoot_fm->InitializeRampToMax(sp_->curr_time, rf_z_max_time);
}

void DoubleSupportStand::oneStep() {
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;

  // Update Floating Base Task
  ctrl_arch_->floating_base_tm->UpdateDesired(sp_->curr_time);
  // Update Foot Task
  ctrl_arch_->rfoot_tm->UpdateZeroAccCmd();
  ctrl_arch_->lfoot_tm->UpdateZeroAccCmd();
  // Update Max Normal Reaction Force
  ctrl_arch_->rfoot_fm->UpdateRampToMax(sp_->curr_time);
  ctrl_arch_->lfoot_fm->UpdateRampToMax(sp_->curr_time);
}

void DoubleSupportStand::lastVisit() {}

bool DoubleSupportStand::endOfState() {
  if (state_machine_time_ > end_time) {
    return true;
  } else {
    return false;
  }
}

StateIdentifier DoubleSupportStand::getNextState() {
  return draco_states::kBalance;
}
