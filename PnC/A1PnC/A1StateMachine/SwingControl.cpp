#include <PnC/A1PnC/A1CtrlArchitecture/A1CtrlArchitecture.hpp>
#include <PnC/A1PnC/A1StateMachine/SwingControl.hpp>
#include <Eigen/Geometry>
#include <cmath>

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

  // Initialize footstep vectors
  front_foot_end_pos = Eigen::VectorXd::Zero(3);
  rear_foot_end_pos = Eigen::VectorXd::Zero(3);

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
  footstepPlanner();

  // Initialize the swing foot trajectory
  if (state_identity_ == A1_STATES::FL_SWING) {
    // Set Front Left Swing Foot Trajectory
    ctrl_arch_->flfoot_trajectory_manager_->initializeSwingFootTrajectory(
        0.0, swing_duration_, sp_->com_vel_des, front_foot_end_pos);
    ctrl_arch_->rrfoot_trajectory_manager_->initializeSwingFootTrajectory(
        0.0, swing_duration_, sp_->com_vel_des, rear_foot_end_pos);
  } else {
    // Set Front Right Swing Foot Trajectory
    ctrl_arch_->frfoot_trajectory_manager_->initializeSwingFootTrajectory(
        0.0, swing_duration_, sp_->com_vel_des, front_foot_end_pos);
    ctrl_arch_->rlfoot_trajectory_manager_->initializeSwingFootTrajectory(
        0.0, swing_duration_, sp_->com_vel_des, rear_foot_end_pos);
  }
}

void SwingControl::footstepPlanner() {

    Eigen::Vector3d p_shoulder, p_symmetry, p_centrifugal, shoulder_location_local_frame;
    Eigen::Matrix<double, 3, 3> Rz_, R_;
    Eigen::Quaternion<double> quat;
    Eigen::Vector3d com_vel_des = ctrl_arch_->floating_base_lifting_up_manager_->com_vel_des_;

    // R_ = robot_->getBodyNodeIsometry(A1BodyNode::trunk).linear().transpose(); // TODO: Is this correct?
    quat = Eigen::Quaternion<double>(robot_->getBodyNodeIsometry(A1BodyNode::trunk).linear().transpose()); // TODO: Is this correct?
    double yaw;
    yaw = std::atan2( 2 * (quat.w() * quat.z() + quat.x() * quat.y()), 
                           1 - 2 * ( std::pow(quat.y(),2) + std::pow(quat.z(),2)));
    Rz_ << cos(yaw), -sin(yaw), 0,
           sin(yaw),  cos(yaw), 0,
                  0,         0, 1;

    if(state_identity_ == A1_STATES::FL_SWING) {
      // p_shoulder = com_pos + Rz(yaw) * shoulder_location_local_frame
      // where Rz(yaw) = rot matrix translating ang vel from 
      // global frame to local frame
      shoulder_location_local_frame = robot_->getBodyNodeIsometry(A1BodyNode::FL_thigh_shoulder).translation() -
                                      robot_->getBodyNodeIsometry(A1BodyNode::trunk).translation();
      p_shoulder = sp_->com_pos + (Rz_ * shoulder_location_local_frame);
      // p_symmetry = (t_stance / 2) * com_velocity + k * (com_vel - com_vel_des) 
      p_symmetry = (swing_duration_ / 2) * sp_->com_vel + 0.05 * (sp_->com_vel - com_vel_des); // TODO: Is t_stance = swing_duration_?
      // p_centrifugal = 0.5 * sqrt(h/g) * (com_vel x ang_vel_des)
      p_centrifugal = 0.5 * std::sqrt(sp_->com_pos[2] / 9.8) * (sp_->com_vel.cross(sp_->base_ang_vel_des)); // TODO: Is h the desired standing height?

      front_foot_end_pos = p_shoulder + p_symmetry + p_centrifugal;

      shoulder_location_local_frame = robot_->getBodyNodeIsometry(A1BodyNode::RR_thigh_shoulder).translation() -
                                      robot_->getBodyNodeIsometry(A1BodyNode::trunk).translation();
      p_shoulder = sp_->com_pos + (Rz_ * shoulder_location_local_frame);

      rear_foot_end_pos = p_shoulder + p_symmetry + p_centrifugal;
    } else {
      // p_shoulder = com_pos + Rz(yaw) * shoulder_location_local_frame
      // where Rz(yaw) = rot matrix translating ang vel from 
      // global frame to local frame
      shoulder_location_local_frame = robot_->getBodyNodeIsometry(A1BodyNode::FR_thigh_shoulder).translation() -
                                      robot_->getBodyNodeIsometry(A1BodyNode::trunk).translation();
      p_shoulder = sp_->com_pos + (Rz_ * shoulder_location_local_frame);
      // p_symmetry = (t_stance / 2) * com_velocity + k * (com_vel - com_vel_des) 
      p_symmetry = (swing_duration_ / 2) * sp_->com_vel + 0.05 * (sp_->com_vel - com_vel_des); // TODO: Is t_stance = swing_duration_?
      // p_centrifugal = 0.5 * sqrt(h/g) * (com_vel x ang_vel_des)
      p_centrifugal = 0.5 * std::sqrt(sp_->com_pos[2] / 9.8) * (sp_->com_vel.cross(sp_->base_ang_vel_des)); // TODO: Is h the desired standing height?

      front_foot_end_pos = p_shoulder + p_symmetry + p_centrifugal;

      shoulder_location_local_frame = robot_->getBodyNodeIsometry(A1BodyNode::RL_thigh_shoulder).translation() -
                                      robot_->getBodyNodeIsometry(A1BodyNode::trunk).translation();
      p_shoulder = sp_->com_pos + (Rz_ * shoulder_location_local_frame);

      rear_foot_end_pos = p_shoulder + p_symmetry + p_centrifugal;
    }
    // Decide which two feet we are planning
    /*if(state_identity_ == A1_STATES::FL_SWING) {
      // FL foot First
      // p_shoulder = body_pos_world + Rz(yaw) * shoulder_pos_body_frame
      // where Rz(yaw) = rotation matrix translating angular velocity in the
      // global frame, ω, to the local (body) coordinate
      Eigen::Vector3d p_shoulder = sp_->com_pos +
            (robot_->getBodyNodeIsometry(A1BodyNode::trunk).linear().transpose()// TODO: get the Rz(yaw))
            * (robot_->getBodyNodeIsometry(A1BodyNode::FL_thigh_shoulder).translation()
            - sp_->com_pos));
      Eigen::Vector3d p_sym = (swing_duration_ / 2) * com_vel_tmp +
                              0.03 * (sp_->com_vel -
                              ctrl_arch_->floating_base_lifting_up_manager_->com_vel_des_);
      Eigen::Vector3d tmp; tmp[0] = 0; tmp[1] = 0; tmp[2] = sp_->x_y_yaw_vel_des[2];
      Eigen::Vector3d p_cent = 0.5 * std::sqrt(0.3 / 9.81) *
                               (com_vel_tmp.cross(tmp));


      front_foot_end_pos = p_shoulder + p_sym + p_cent;

      p_shoulder = sp_->com_pos + (robot_->getBodyNodeIsometry(A1BodyNode::trunk).linear().transpose() *
                   (robot_->getBodyNodeIsometry(A1BodyNode::RR_thigh_shoulder).translation() -
                   sp_->com_pos));
      rear_foot_end_pos = p_shoulder + p_sym + p_cent;
    } else {
      Eigen::Vector3d p_shoulder = sp_->com_pos +
            (robot_->getBodyNodeIsometry(A1BodyNode::trunk).linear().transpose()// TODO: get the Rz(yaw))
            * (robot_->getBodyNodeIsometry(A1BodyNode::FR_thigh_shoulder).translation()
            - sp_->com_pos));
      Eigen::Vector3d p_sym = (swing_duration_ / 2) * com_vel_tmp +
                              0.03 * (sp_->com_vel -
                              ctrl_arch_->floating_base_lifting_up_manager_->com_vel_des_);
      Eigen::Vector3d tmp; tmp[0] = 0; tmp[1] = 0; tmp[2] = sp_->x_y_yaw_vel_des[2];
      Eigen::Vector3d p_cent = 0.5 * std::sqrt(0.3 / 9.81) *
                               (com_vel_tmp.cross(tmp));


      front_foot_end_pos = p_shoulder + p_sym + p_cent;

      p_shoulder = sp_->com_pos + (robot_->getBodyNodeIsometry(A1BodyNode::trunk).linear().transpose() *
                   (robot_->getBodyNodeIsometry(A1BodyNode::RL_thigh_shoulder).translation() -
                   sp_->com_pos));
      rear_foot_end_pos = p_shoulder + p_sym + p_cent;
    }*/

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
  // std::cout << "swing state machine time = " << state_machine_time_ << std::endl;
  // std::cout << "swing end time = " << end_time_ << std::endl;
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
