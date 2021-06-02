#include <PnC/A1PnC/A1CtrlArchitecture/A1CtrlArchitecture.hpp>
#include <PnC/A1PnC/A1StateMachine/SwingControl.hpp>
#include <Eigen/Geometry>
#include <cmath>
#include <PnC/A1PnC/A1Definition.hpp>

SwingControl::SwingControl(const StateIdentifier state_identifier_in,
                           const int _leg_side,
                           A1ControlArchitecture* _ctrl_arch,
                           RobotSystem* _robot)
    : StateMachine(state_identifier_in, _robot) {
  myUtils::pretty_constructor(2, "SM: Swing Control");

  // Defaults
  // Percent of end_time to check if there are early contacts
  swing_time_percent_early_contact_check_ = 0.75;
  // Force Threshold to check for early contact
  early_contact_force_threshold_ = 10;  // Newtons

  // Set Pointer to Control Architecture
  ctrl_arch_ = ((A1ControlArchitecture*)_ctrl_arch);
  taf_container_ = ctrl_arch_->taf_container_;
  // Get State Provider
  sp_ = A1StateProvider::getStateProvider(robot_);

  // Initialize footstep planning vars
  front_foot_end_pos = Eigen::VectorXd::Zero(3);
  rear_foot_end_pos = Eigen::VectorXd::Zero(3);
  p_sh = Eigen::VectorXd::Zero(3);
  p_sym = Eigen::VectorXd::Zero(3);
  p_cent = Eigen::VectorXd::Zero(3);
  rot_ = Eigen::MatrixXd::Zero(3,3);
  yaw = 0;

  test_counter = 0;

}

SwingControl::~SwingControl() {}


void SwingControl::firstVisit() {

  // Manage contact and task lists depending on step side
  taf_container_->task_list_.clear();
  taf_container_->contact_list_.clear();
  taf_container_->task_list_.push_back(taf_container_->com_task_);
  taf_container_->task_list_.push_back(taf_container_->base_ori_task_);
  if (state_identity_ == A1_STATES::FR_HALF_SWING) {
    std::cout << "[Front Right Foot First Half Swing]" << std::endl;
    taf_container_->task_list_.push_back(taf_container_->frfoot_pos_task_);
    taf_container_->task_list_.push_back(taf_container_->rlfoot_pos_task_);
    taf_container_->contact_list_.push_back(taf_container_->flfoot_contact_);
    taf_container_->contact_list_.push_back(taf_container_->rrfoot_contact_);
  } else if (state_identity_ == A1_STATES::FL_HALF_SWING) {
    std::cout << "[Front Left Foot First Half Swing]" << std::endl;
    taf_container_->task_list_.push_back(taf_container_->flfoot_pos_task_);
    taf_container_->task_list_.push_back(taf_container_->rrfoot_pos_task_);
    taf_container_->contact_list_.push_back(taf_container_->frfoot_contact_);
    taf_container_->contact_list_.push_back(taf_container_->rlfoot_contact_);
  } else if (state_identity_ == A1_STATES::FR_FINAL_SWING) {
    std::cout << "[Front Right Foot Second Half Swing]" << std::endl;
    taf_container_->task_list_.push_back(taf_container_->frfoot_pos_task_);
    taf_container_->task_list_.push_back(taf_container_->rlfoot_pos_task_);
    taf_container_->contact_list_.push_back(taf_container_->flfoot_contact_);
    taf_container_->contact_list_.push_back(taf_container_->rrfoot_contact_);
  } else {
    std::cout << "[Front Left Foot Second Half Swing]" << std::endl;
    taf_container_->task_list_.push_back(taf_container_->flfoot_pos_task_);
    taf_container_->task_list_.push_back(taf_container_->rrfoot_pos_task_);
    taf_container_->contact_list_.push_back(taf_container_->frfoot_contact_);
    taf_container_->contact_list_.push_back(taf_container_->rlfoot_contact_);
  }
  // Set control Starting time
  ctrl_start_time_ = sp_->curr_time;
  end_time_ = swing_duration_/2.;

  // In this implementation, we use the footstep planner of Mingyo
  // donghyunFootstepPlanner();
  // Grab the values input from Mingyo through A1Command
  front_foot_end_pos = sp_->next_front_foot_location;
  rear_foot_end_pos = sp_->next_rear_foot_location;

  // Initialize the swing foot trajectory
  if (state_identity_ == A1_STATES::FL_HALF_SWING ||
      state_identity_ == A1_STATES::FL_FINAL_SWING) {
    // Set Front Left Swing Foot Trajectory
    ctrl_arch_->flfoot_trajectory_manager_->initializeSwingFootTrajectory(
        0.0, swing_duration_/2., sp_->com_vel_des, front_foot_end_pos);
    ctrl_arch_->rrfoot_trajectory_manager_->initializeSwingFootTrajectory(
        0.0, swing_duration_/2., sp_->com_vel_des, rear_foot_end_pos);
  } else {
    // Set Front Right Swing Foot Trajectory
    ctrl_arch_->frfoot_trajectory_manager_->initializeSwingFootTrajectory(
        0.0, swing_duration_/2., sp_->com_vel_des, front_foot_end_pos);
    ctrl_arch_->rlfoot_trajectory_manager_->initializeSwingFootTrajectory(
        0.0, swing_duration_/2., sp_->com_vel_des, rear_foot_end_pos);
  }
}



/*void SwingControl::donghyunFootstepPlanner() {
  // Raibert style footstep planner for a quadruped
  // From his paper https://arxiv.org/pdf/1909.06586
  // ri_cmd = p_sh + p_sym + p_cent
  // p_sh = pk + Rz(yaw) * l_i = body_pos_world_time_k + rot_global_to_local * shoulder_i_pos_body
  // --> p_sh = shoulder_pos_world
  // p_sym = (t_stance / 2) * v + k * (v - v_cmd) (Raibert heuristic)
  // p_cent = 0.5 * sqrt(h/g) * (v x omega)

  Eigen::VectorXd tmp; tmp = Eigen::VectorXd::Zero(3); tmp[0] = 0.1;

  // In trot we plan either FL+RR or FR+RL
  if(state_identity_ == A1_STATES::FL_SWING) {
    p_sh = robot_->getBodyNodeIsometry(A1BodyNode::FL_thigh_shoulder).translation() + tmp;
    p_sym = ((0.05 / 2) * sp_->com_vel) + (0.03 * (sp_->com_vel - sp_->com_vel_des));
    // p_sym = ((0.1) * sp_->com_vel) + (0.03 * (sp_->com_vel - sp_->com_vel_des));
    p_cent = 0.5 * std::sqrt(0.25/9.8) * (sp_->com_vel.cross(sp_->base_ang_vel_des));

    front_foot_end_pos = p_sh + p_sym + p_cent;

    p_sh = robot_->getBodyNodeIsometry(A1BodyNode::RR_thigh_shoulder).translation() + tmp;
    rear_foot_end_pos = p_sh + p_sym + p_cent;
  } else {
    p_sh = robot_->getBodyNodeIsometry(A1BodyNode::FR_thigh_shoulder).translation() + tmp;
    p_sym = ((0.05 / 2) * sp_->com_vel) + (0.03 * (sp_->com_vel - sp_->com_vel_des));
    // p_sym = ((0.1) * sp_->com_vel) + (0.03 * (sp_->com_vel - sp_->com_vel_des));
    p_cent = 0.5 * std::sqrt(0.25/9.8) * (sp_->com_vel.cross(sp_->base_ang_vel_des));

    front_foot_end_pos = p_sh + p_sym + p_cent;

    p_sh = robot_->getBodyNodeIsometry(A1BodyNode::RL_thigh_shoulder).translation() + tmp;
    rear_foot_end_pos = p_sh + p_sym + p_cent;
  }

  // TEST FOOTSTEP PLANNER
  double yaw = sp_->base_ang_vel_des[2] * A1Aux::servo_rate * test_counter;
  if(state_identity_ == A1_STATES::FL_SWING) {
    p_sh = robot_->getBodyNodeIsometry(A1BodyNode::FL_thigh_shoulder).translation()
           + (sp_->com_vel_des * A1Aux::servo_rate * test_counter);
    p_sh[0] = p_sh[0]*cos(yaw) + p_sh[1]*sin(yaw);
    p_sh[1] = -p_sh[0]*sin(yaw) + p_sh[1]*cos(yaw);
    p_sym = ((0.05 / 2) * sp_->com_vel_des); // + (0.03 * (sp_->com_vel - sp_->com_vel_des));
    p_cent = 0.5 * std::sqrt(0.25/9.8) * (sp_->com_vel_des.cross(sp_->base_ang_vel_des));

    front_foot_end_pos = p_sh + p_sym + p_cent;

    p_sh = robot_->getBodyNodeIsometry(A1BodyNode::RR_thigh_shoulder).translation()
           + (sp_->com_vel_des * A1Aux::servo_rate * test_counter);
    p_sh[0] = p_sh[0]*cos(yaw) + p_sh[1]*sin(yaw);
    p_sh[1] = -p_sh[0]*sin(yaw) + p_sh[1]*cos(yaw);
    rear_foot_end_pos = p_sh + p_sym + p_cent;

    sp_->flfoot_landing = front_foot_end_pos;
    sp_->rrfoot_landing = rear_foot_end_pos;
  } else {
    p_sh = robot_->getBodyNodeIsometry(A1BodyNode::FR_thigh_shoulder).translation()
           + (sp_->com_vel_des * A1Aux::servo_rate * test_counter);
    p_sh[0] = p_sh[0]*cos(yaw) + p_sh[1]*sin(yaw);
    p_sh[1] = -p_sh[0]*sin(yaw) + p_sh[1]*cos(yaw);

    p_sym = ((0.05 / 2) * sp_->com_vel_des); //+ (0.03 * (sp_->com_vel - sp_->com_vel_des));
    p_cent = 0.5 * std::sqrt(0.25/9.8) * (sp_->com_vel_des.cross(sp_->base_ang_vel_des));

    front_foot_end_pos = p_sh + p_sym + p_cent;

    p_sh = robot_->getBodyNodeIsometry(A1BodyNode::RL_thigh_shoulder).translation()
           + (sp_->com_vel_des * A1Aux::servo_rate * test_counter);
    p_sh[0] = p_sh[0]*cos(yaw) + p_sh[1]*sin(yaw);
    p_sh[1] = -p_sh[0]*sin(yaw) + p_sh[1]*cos(yaw);
    rear_foot_end_pos = p_sh + p_sym + p_cent;
    sp_->frfoot_landing = front_foot_end_pos;
    sp_->rlfoot_landing = rear_foot_end_pos;
  }
  // END TEST FOOTSTEP PLANNER
}*/



void SwingControl::_taskUpdate() {
  // =========================================================================
  // Foot
  // =========================================================================
  if (state_identity_ == A1_STATES::FL_HALF_SWING || state_identity_ == A1_STATES::FL_FINAL_SWING) {
    // Set Front Left Swing Foot Trajectory
    ctrl_arch_->flfoot_trajectory_manager_->updateSwingFootDesired(
        state_machine_time_);
    ctrl_arch_->rrfoot_trajectory_manager_->updateSwingFootDesired(
        state_machine_time_);
  } else {
    // Set Right Swing FOot Trajectory. Hold Other foot in place.
    ctrl_arch_->frfoot_trajectory_manager_->updateSwingFootDesired(
        state_machine_time_);
    ctrl_arch_->rlfoot_trajectory_manager_->updateSwingFootDesired(
        state_machine_time_);
  }

  // =========================================================================
  // Floating Base
  // =========================================================================
  ctrl_arch_->floating_base_lifting_up_manager_->updateFloatingBaseWalkingDesired(
                                    sp_->com_pos,
                                    sp_->x_y_yaw_vel_des);
}

void SwingControl::oneStep() {
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;
  _taskUpdate();
}

void SwingControl::lastVisit() {}

bool SwingControl::endOfState() {
  // std::cout << "swing state machine time = " << state_machine_time_ << std::endl;
  // std::cout << "swing end time = " << end_time_ << std::endl;
  if (state_machine_time_ >= end_time_) { // && sp_->b_flfoot_contact && sp_->b_frfoot_contact) {
    return true;
  } else {
    /*if (state_machine_time_ >=
        swing_time_percent_early_contact_check_ * end_time_) {
      if (leg_side_ == (LEFT_ROBOT_SIDE)) {
        if (fabs(sp_->l_rf[5]) >= early_contact_force_threshold_) {
          std::cout << "early left foot contact" << std::endl;
          std::cout << "state_machine_time_ :" << state_machine_time_ << " / "
                    << end_time_ << std::endl;
          myUtils::pretty_print(sp_->l_rf, std::cout, "Left Wrench");
          return true;
        }
      } else {
        if (fabs(sp_->r_rf[5]) >= early_contact_force_threshold_) {
          std::cout << "early right foot contact" << std::endl;
          std::cout << "state_machine_time_ :" << state_machine_time_ << " / "
                    << end_time_ << std::endl;
          myUtils::pretty_print(sp_->r_rf, std::cout, "Right Wrench");
          return true;
        }
      }
    }*/
    return false;
  }
}

// bool SwingControl::endOfState() {
// if (state_machine_time_ >= end_time_) {
// return true;
//} else {
// if (state_machine_time_ >=
// swing_time_percent_early_contact_check_ * end_time_) {
// if (leg_side_ == (LEFT_ROBOT_SIDE)) {
// if (sp_->b_lfoot_contact == 1) {
// std::cout << "early left foot contact" << std::endl;
// std::cout << "state_machine_time_ :" << state_machine_time_ << " / "
//<< end_time_ << std::endl;
// return true;
//}
//} else {
// if (sp_->b_rfoot_contact == 1) {
// std::cout << "state_machine_time_ :" << state_machine_time_ << " / "
//<< end_time_ << std::endl;
// std::cout << "early right foot contact" << std::endl;
// return true;
//}
//}
//}
// return false;
//}
//}

StateIdentifier SwingControl::getNextState() {
  if(state_identity_ == A1_STATES::FR_FINAL_SWING){
    return A1_STATES::FL_CONTACT_TRANSITION_START;
  } else if(state_identity_ == A1_STATES::FL_FINAL_SWING) {
    return A1_STATES::FR_CONTACT_TRANSITION_START;
  } else if (state_identity_ == A1_STATES::FR_HALF_SWING) {
    return state_identity_ == A1_STATES::FR_FINAL_SWING;
  } else {
    return state_identity_ == A1_STATES::FL_FINAL_SWING;
  }

}

void SwingControl::initialization(const YAML::Node& node) {
  try {
    myUtils::readParameter(node, "swing_duration", swing_duration_);
    myUtils::readParameter(node, "swing_time_percent_early_contact_check",
                           swing_time_percent_early_contact_check_);
    myUtils::readParameter(node, "early_contact_force_threshold",
                           early_contact_force_threshold_);
  } catch (std::runtime_error& e) {
    std::cout << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
    exit(0);
  }
}
