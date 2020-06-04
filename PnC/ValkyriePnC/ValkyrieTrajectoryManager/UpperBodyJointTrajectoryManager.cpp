#include <PnC/ValkyriePnC/ValkyrieTrajectoryManager/UpperBodyJointTrajectoryManager.hpp>
#include <PnC/WBC/BasicTask.hpp>

UpperBodyJointTrajectoryManager::UpperBodyJointTrajectoryManager(
    Task* _upper_body_task, RobotSystem* _robot)
    : TrajectoryManagerBase(_robot) {
  myUtils::pretty_constructor(2, "Trajectory Manager: Upper Body Joints");
  // Set Task
  upper_body_task_ = _upper_body_task;

  // Get State Provider
  sp_ = ValkyrieStateProvider::getStateProvider(robot_);

  raising_arm = false;
  lowering_arm = false;

  // Initialize defaults:
  int offset = ValkyrieDoF::torsoYaw;
  rarm_joint_index_ = ValkyrieDoF::rightShoulderPitch - offset;
  arm_trajectory_duration_ = 1.0;  // seconds
  raise_target_value_ = -1.5;      // radians
  lower_target_value_ = 0.0;       // radians

  // Initialize number of joints
  number_of_joints_ = upper_body_task_->getDim();

  t_start_ = 0.0;
}

UpperBodyJointTrajectoryManager::~UpperBodyJointTrajectoryManager() {}

void UpperBodyJointTrajectoryManager::initializeRaiseRightArmNow() {
  std::cout << "[UpperBodyJointTrajectoryManager] intiialize raising arm"
            << std::endl;
  // get initial time and current position
  t_start_ = sp_->curr_time;
  pos_ini_ = sp_->q.tail(number_of_joints_);
  vel_ini_ = sp_->qdot.tail(number_of_joints_);

  // set flag
  raising_arm = true;
  lowering_arm = false;
}
void UpperBodyJointTrajectoryManager::initializeLowerRightArmNow() {
  std::cout << "[UpperBodyJointTrajectoryManager] intiialize lower arm"
            << std::endl;
  // get initial time and current position
  t_start_ = sp_->curr_time;
  pos_ini_ = sp_->q.tail(number_of_joints_);
  vel_ini_ = sp_->qdot.tail(number_of_joints_);

  // set flag
  raising_arm = false;
  lowering_arm = true;
}

void UpperBodyJointTrajectoryManager::getCurrentDesired() {
  pos_des_ = upper_body_task_->pos_des_;
  vel_des_ = upper_body_task_->vel_des_;
  acc_des_ = upper_body_task_->acc_des_;
}

void UpperBodyJointTrajectoryManager::computeArmTrajectories() {
  double t_local = (sp_->curr_time - t_start_);
  double target = lower_target_value_;

  // Update desireds ----

  // Select target whether it is raising or lowering the arm.
  if (raising_arm) {
    target = raise_target_value_;
  } else if (lowering_arm) {
    target = lower_target_value_;
  }

  if (raising_arm || lowering_arm) {
    // Update if interval is valid
    if (t_local <= arm_trajectory_duration_) {
      pos_des_[rarm_joint_index_] =
          myUtils::smooth_changing(pos_ini_[rarm_joint_index_], target,
                                   arm_trajectory_duration_, t_local);
      vel_des_[rarm_joint_index_] = myUtils::smooth_changing_vel(
          vel_ini_[rarm_joint_index_], 0.0, arm_trajectory_duration_, t_local);
      acc_des_[rarm_joint_index_] = 0.0;
    } else {
      // Disable flags if time interval exceeds duration
      raising_arm = false;
      lowering_arm = false;
    }
  }
}

void UpperBodyJointTrajectoryManager::updateDesired() {
  // Get current task desired.
  getCurrentDesired();
  // Modify only particular values
  computeArmTrajectories();
  // myUtils::pretty_print(pos_des_, std::cout, "jpos_des_");
  // Update new desireds
  upper_body_task_->updateDesired(pos_des_, vel_des_, acc_des_);
}

void UpperBodyJointTrajectoryManager::paramInitialization(
    const YAML::Node& node) {
  try {
    // myUtils::readParameter(node, "target", target);

  } catch (std::runtime_error& e) {
    std::cout << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
    exit(0);
  }
}

