#include <PnC/TrajectoryManager/PointFootTrajectoryManager.hpp>

PointFootTrajectoryManager::PointFootTrajectoryManager(Task* _foot_pos_task,
                                                       RobotSystem* _robot)
    : TrajectoryManagerBase(_robot) {
  myUtils::pretty_constructor(2, "TrajectoryManager: PointFoot");
  // Set Linear and Orientation Foot task
  foot_pos_task_ = _foot_pos_task;

  // Assume that both tasks use the same link id.
  link_idx_ = static_cast<BasicTask*>(foot_pos_task_)->getLinkID();

  // Initialize member variables
  foot_pos_des_.setZero();
  foot_vel_des_.setZero();
  foot_acc_des_.setZero();

  swing_height_ = 0.05;  // 5cm default
}

PointFootTrajectoryManager::~PointFootTrajectoryManager() {}

void PointFootTrajectoryManager::paramInitialization(const YAML::Node& node) {
  try {
    myUtils::readParameter(node, "swing_height", swing_height_);

  } catch (std::runtime_error& e) {
    std::cout << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
    exit(0);
  }
}

void PointFootTrajectoryManager::useCurrent() {
  // Update desired to use current foot pose
  foot_pos_des_ = robot_->getBodyNodeCoMIsometry(link_idx_).translation();
  foot_vel_des_ = robot_->getBodyNodeSpatialVelocity(link_idx_).tail(3);
  // foot_vel_des_.setZero();
  foot_acc_des_.setZero();
  updateDesired();
}

void PointFootTrajectoryManager::updateDesired() {
  foot_pos_task_->updateDesired(foot_pos_des_, foot_vel_des_, foot_acc_des_);
}

// Initialize the swing foot trajectory
void PointFootTrajectoryManager::initializeSwingFootTrajectory(
    const double _start_time, const double _swing_duration,
    const Footstep& _landing_foot) {}

// Computes the swing foot
void PointFootTrajectoryManager::computeSwingFoot(const double current_time) {}

void PointFootTrajectoryManager::updateSwingFootDesired(
    const double current_time) {}
