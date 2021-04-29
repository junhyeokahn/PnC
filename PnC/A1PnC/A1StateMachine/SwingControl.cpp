#include <PnC/A1PnC/A1CtrlArchitecture/A1CtrlArchitecture.hpp>
#include <PnC/A1PnC/A1StateMachine/SwingControl.hpp>

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

  // Set Leg Side
  leg_side_ = _leg_side;
}

SwingControl::~SwingControl() {}

void SwingControl::firstVisit() {

  // Manage contact and task lists depending on step side
  taf_container_->task_list_.clear();
  taf_container_->contact_list_.clear();
  taf_container_->task_list_.push_back(taf_container_->com_task_);
  taf_container_->task_list_.push_back(taf_container_->base_ori_task_);
  if (state_identity_ == A1_STATES::FR_SWING) {
    std::cout << "[Front Right Foot Swing]" << std::endl;
    taf_container_->task_list_.push_back(taf_container_->frfoot_pos_task_);
    taf_container_->task_list_.push_back(taf_container_->rlfoot_pos_task_);
    taf_container_->contact_list_.push_back(taf_container_->flfoot_contact_);
    taf_container_->contact_list_.push_back(taf_container_->rrfoot_contact_);
  } else {
    std::cout << "[Front Left Foot Swing]" << std::endl;
    taf_container_->task_list_.push_back(taf_container_->flfoot_pos_task_);
    taf_container_->task_list_.push_back(taf_container_->rrfoot_pos_task_);
    taf_container_->contact_list_.push_back(taf_container_->frfoot_contact_);
    taf_container_->contact_list_.push_back(taf_container_->rlfoot_contact_);
  }
  // Set control Starting time
  ctrl_start_time_ = sp_->curr_time;
  end_time_ = swing_duration_;

  // TODO: FOOTSTEP PLANNING --> pass to initializeSwingFootTrajectory

  // Initialize the swing foot trajectory
  if (state_identity_ == A1_STATES::FL_SWING) {
    // Set Front Left Swing Foot Trajectory
    ctrl_arch_->flfoot_trajectory_manager_->initializeSwingFootTrajectory(
        0.0, swing_duration_, sp_->com_vel_des);
    ctrl_arch_->rrfoot_trajectory_manager_->initializeSwingFootTrajectory(
        0.0, swing_duration_, sp_->com_vel_des);
  } else {
    // Set Front Right Swing Foot Trajectory
    ctrl_arch_->frfoot_trajectory_manager_->initializeSwingFootTrajectory(
        0.0, swing_duration_, sp_->com_vel_des);
    ctrl_arch_->rlfoot_trajectory_manager_->initializeSwingFootTrajectory(
        0.0, swing_duration_, sp_->com_vel_des);
  }
}

void SwingControl::_taskUpdate() {
  // =========================================================================
  // Foot
  // =========================================================================
  if (state_identity_ == A1_STATES::FL_SWING) {
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
  std::cout << "swing state machine time = " << state_machine_time_ << std::endl;
  std::cout << "swing end time = " << end_time_ << std::endl;
  if (state_machine_time_ >= end_time_) {
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
  if(state_identity_ == A1_STATES::FR_SWING){
    return A1_STATES::FL_CONTACT_TRANSITION_START;
  }
  else { return A1_STATES::FR_CONTACT_TRANSITION_START; }

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
