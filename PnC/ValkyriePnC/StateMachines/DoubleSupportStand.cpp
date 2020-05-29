#include <PnC/ValkyriePnC/StateMachines/DoubleSupportStand.hpp>
#include <PnC/ValkyriePnC/CtrlArchitectures/ValkyrieControlArchitecture.hpp>

DoubleSupportStand::DoubleSupportStand(const StateIdentifier state_identifier_in, ValkyrieControlArchitecture* _ctrl_arch, RobotSystem* _robot) : 
               StateMachine(state_identifier_in, _robot) {
  myUtils::pretty_constructor(2, "SM: Double Support Stand");

  // Set Pointer to Control Architecture
  val_ctrl_arch_ = ((ValkyrieControlArchitecture*) _ctrl_arch);
  taf_container_ = val_ctrl_arch_->taf_container_;

  // Get State Provider
  sp_ = ValkyrieStateProvider::getStateProvider(robot_);

  // Default time to max normal force
  time_to_max_normal_force_ = 0.1;

  // To Do: Belongs to trajectory manager.
  // COM
  ini_com_pos_ = Eigen::VectorXd::Zero(3); 
  des_com_pos_ = Eigen::VectorXd::Zero(3);
  des_com_vel_ = Eigen::VectorXd::Zero(3);
  des_com_acc_ = Eigen::VectorXd::Zero(3);
  com_pos_target_ = Eigen::VectorXd::Zero(3);
}

DoubleSupportStand::~DoubleSupportStand(){
}


void DoubleSupportStand::firstVisit(){
  std::cout << "Start [Double Support Stand]" << std::endl;

  ctrl_start_time_ = sp_->curr_time;
  // =========================================================================
  // Set CoM Trajectory
  // =========================================================================
  ini_com_pos_ = robot_->getCoMPosition();

  Eigen::Vector3d lfoot_pos = robot_->getBodyNodeCoMIsometry(ValkyrieBodyNode::leftCOP_Frame).translation();
  Eigen::Vector3d rfoot_pos = robot_->getBodyNodeCoMIsometry(ValkyrieBodyNode::rightCOP_Frame).translation();
  des_com_pos_ = 0.5*(lfoot_pos + rfoot_pos) + com_pos_target_;

  _SetBspline(ini_com_pos_,des_com_pos_);

  ini_pelvis_quat_ = Eigen::Quaternion<double>(robot_->getBodyNodeIsometry(ValkyrieBodyNode::pelvis).linear());

  // =========================================================================
  // Pelvis Ori Task: Maintain Starting Orientation
  // =========================================================================
  Eigen::VectorXd des_pelvis_quat = Eigen::VectorXd::Zero(4);
  des_pelvis_quat << ini_pelvis_quat_.w(),ini_pelvis_quat_.x(), ini_pelvis_quat_.y(),
                      ini_pelvis_quat_.z();
  taf_container_->pelvis_ori_task_->updateDesired(des_pelvis_quat, Eigen::VectorXd::Zero(3),Eigen::VectorXd::Zero(3));

  // =========================================================================
  // Set Angular Momentum Tasks
  // =========================================================================
  Eigen::VectorXd zero3 = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd des_ang_momentum = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd des_ang_momentum_rate = Eigen::VectorXd::Zero(3);
  taf_container_->ang_momentum_task_->updateDesired(zero3, des_ang_momentum, des_ang_momentum_rate);

  // =========================================================================
  // Joint Pos Task
  // =========================================================================
  Eigen::VectorXd jpos_des = sp_->jpos_ini;
  taf_container_->upper_body_task_->updateDesired(jpos_des.tail(taf_container_->upper_body_joint_indices_.size()),
                                                  Eigen::VectorXd::Zero(Valkyrie::n_adof),
                                                  Eigen::VectorXd::Zero(Valkyrie::n_adof));
  // =========================================================================  
  // Initialize Reaction Force Ramp to Max 
  // =========================================================================
  val_ctrl_arch_->lfoot_max_normal_force_manager_->initializeRampToMax(0.0, time_to_max_normal_force_);
  val_ctrl_arch_->rfoot_max_normal_force_manager_->initializeRampToMax(0.0, time_to_max_normal_force_);
}

void DoubleSupportStand::_taskUpdate(){
  // =========================================================================
  // Update CoM Task
  // =========================================================================
  _GetBsplineTrajectory();
  for (int i = 0; i < 3; ++i) {
      (sp_->com_pos_des)[i] = des_com_pos_[i];
      (sp_->com_vel_des)[i] = des_com_vel_[i];
  }
  taf_container_->com_task_->updateDesired(des_com_pos_, des_com_vel_, des_com_acc_);

  // =========================================================================
  // Set Foot Motion Tasks
  // =========================================================================
  val_ctrl_arch_->rfoot_trajectory_manager_->useCurrent();
  val_ctrl_arch_->lfoot_trajectory_manager_->useCurrent();
}

void DoubleSupportStand::oneStep(){  
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;

  // Compute and update new maximum reaction forces
  val_ctrl_arch_->lfoot_max_normal_force_manager_->updateRampToMaxDesired(state_machine_time_);
  val_ctrl_arch_->rfoot_max_normal_force_manager_->updateRampToMaxDesired(state_machine_time_);

  _taskUpdate();
}

void DoubleSupportStand::lastVisit(){  
}

bool DoubleSupportStand::endOfState(){  
  if (state_machine_time_ > end_time_){
    std::cout << "[DoubleSupportStand] State End" << std::endl;
    return true;
  }
  return false;
} 

StateIdentifier DoubleSupportStand::getNextState(){
  return VALKYRIE_STATES::BALANCE;
}


void DoubleSupportStand::_SetBspline(const Eigen::VectorXd st_pos,
                          const Eigen::VectorXd des_pos) {
  // Trajectory Setup
  double init[9];
  double fin[9];
  double** middle_pt = new double*[1];
  middle_pt[0] = new double[3];
  Eigen::Vector3d middle_pos;

  middle_pos = (st_pos + des_pos) / 2.;

  // Initial and final position & velocity & acceleration
  for (int i(0); i < 3; ++i) {
    // Initial
    init[i] = st_pos[i];
    init[i + 3] = 0.;
    init[i + 6] = 0.;
    // Final
    fin[i] = des_pos[i];
    fin[i + 3] = 0.;
    fin[i + 6] = 0.;
    // Middle
    middle_pt[0][i] = middle_pos[i];
  }
  // TEST
  //fin[5] = amplitude_[] * omega_;
  com_traj_.SetParam(init, fin, middle_pt, end_time_/2.0);

  delete[] * middle_pt;
  delete[] middle_pt;
}

void DoubleSupportStand::_GetBsplineTrajectory(){
  double pos[3];
  double vel[3];
  double acc[3];

  com_traj_.getCurvePoint(state_machine_time_, pos);
  com_traj_.getCurveDerPoint(state_machine_time_, 1, vel);
  com_traj_.getCurveDerPoint(state_machine_time_, 2, acc);

  for (int i(0); i < 3; ++i) {
    des_com_pos_[i] = pos[i];
    des_com_vel_[i] = vel[i];
    des_com_acc_[i] = acc[i];
  }
}


void DoubleSupportStand::initialization(const YAML::Node& node){
    try {
        myUtils::readParameter(node,"target_pos_duration",end_time_);
        myUtils::readParameter(node, "time_to_max_normal_force", time_to_max_normal_force_);
        myUtils::readParameter(node, "com_pos_target", com_pos_target_);

     } catch(std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
        exit(0);
    }
}