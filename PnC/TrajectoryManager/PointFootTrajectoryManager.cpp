#include <PnC/TrajectoryManager/PointFootTrajectoryManager.hpp>

PointFootTrajectoryManager::PointFootTrajectoryManager(Task* _foot_pos_task,
                                                       RobotSystem* _robot)
    : TrajectoryManagerBase(_robot) {
  myUtils::pretty_constructor(2, "TrajectoryManager: PointFoot");
  // Set Linear Foot task
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
    const double _start_time, const double _swing_duration, Eigen::VectorXd com_vel_des,
    const Eigen::VectorXd _end_foot_pos) {
  swing_start_time_ = _start_time;
  swing_duration_ = _swing_duration;

  Eigen::Vector3d start_foot_pos =
    robot_->getBodyNodeCoMIsometry(link_idx_).translation();
  Eigen::Vector3d end_foot_pos = _end_foot_pos;
  end_foot_pos[2] = start_foot_pos[2];

  Eigen::Vector3d midfoot_pos = 0.5 * (end_foot_pos + start_foot_pos);
  midfoot_pos[2] += swing_height_;

  myUtils::pretty_print(start_foot_pos, std::cout, "start_foot_pos");
  myUtils::pretty_print(midfoot_pos, std::cout, "midfoot_pos");
  myUtils::pretty_print(end_foot_pos, std::cout, "end_foot_pos");

  Eigen::Vector3d mid_swing_velocity =
      (end_foot_pos - start_foot_pos) / swing_duration_;

  // Construct Position trajectories
  pos_traj_init_to_mid_.initialize(start_foot_pos,
                                   Eigen::Vector3d::Zero(3), midfoot_pos,
                                   mid_swing_velocity);
  pos_traj_mid_to_end_.initialize(midfoot_pos, mid_swing_velocity,
                                  end_foot_pos,
                                  Eigen::Vector3d::Zero(3));

}

// Computes the swing foot
void PointFootTrajectoryManager::computeSwingFoot(const double current_time) {
  // Compute progression variable
  double s = (current_time - swing_start_time_) / swing_duration_;
  // Get foot position and its derivatives
  if (s <= 0.5) {  // 0.0 <= s < 0.5 use the first trajectory
    // scale back to 1.0
    s = 2.0 * s;
    foot_pos_des_ = pos_traj_init_to_mid_.evaluate(s);
    foot_vel_des_ = (pos_traj_init_to_mid_.evaluateFirstDerivative(s)) /
                    ( swing_duration_*0.5 );
    foot_acc_des_ = foot_acc_des_ = (pos_traj_init_to_mid_.evaluateSecondDerivative(s)) /
                    ( swing_duration_*0.5 );
    // myUtils::pretty_print(foot_vel_des_, std::cout, "foot vel des");
    // std::cout << "s = " << s << std::endl;
  } else {  // 0.5 <= s < 1.0 use the second trajectory
    // scale back to 1.0 after the offset
    s = 2.0 * (s - 0.5);
    foot_pos_des_ = pos_traj_mid_to_end_.evaluate(s);
    foot_vel_des_ = (pos_traj_mid_to_end_.evaluateFirstDerivative(s)) /
                    ( swing_duration_*0.5 );
    foot_acc_des_ = (pos_traj_mid_to_end_.evaluateSecondDerivative(s)) /
                    ( swing_duration_*0.5 );
  }


}

void PointFootTrajectoryManager::updateSwingFootDesired(
    const double current_time) {
  computeSwingFoot(current_time);
  updateDesired();
}
