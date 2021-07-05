#include "pnc/whole_body_controllers/ihwbc/joint_integrator.hpp"

JointIntegrator::JointIntegrator(int num_joints_in, double dt_in) {
  util::PrettyConstructor(3, "WBC Joint Integrator");
  n_joints_ = num_joints_in;
  vel_ = Eigen::VectorXd::Zero(n_joints_);
  pos_ = Eigen::VectorXd::Zero(n_joints_);
  vel_min_ = Eigen::VectorXd::Zero(n_joints_);
  vel_max_ = Eigen::VectorXd::Zero(n_joints_);
  pos_min_ = Eigen::VectorXd::Zero(n_joints_);
  pos_max_ = Eigen::VectorXd::Zero(n_joints_);
  pos_max_error_ = Eigen::VectorXd::Zero(n_joints_);
  setDt(dt_in);
  setDefaultSaturation();
  setVelocityFrequencyCutOff(default_vel_freq_cutoff_);
  setPositionFrequencyCutOff(default_pos_freq_cutoff_);
  b_initialized = false;
}

JointIntegrator::JointIntegrator(const int num_joints_in,
                                 const double vel_cutoff_in,
                                 const double pos_cutoff_in,
                                 const double dt_in) {
  std::cout << "a" << std::endl;
  util::PrettyConstructor(3, "IHWBC Joint Integrator");
  n_joints_ = num_joints_in;
  vel_ = Eigen::VectorXd::Zero(n_joints_);
  pos_ = Eigen::VectorXd::Zero(n_joints_);
  vel_min_ = Eigen::VectorXd::Zero(n_joints_);
  vel_max_ = Eigen::VectorXd::Zero(n_joints_);
  pos_min_ = Eigen::VectorXd::Zero(n_joints_);
  pos_max_ = Eigen::VectorXd::Zero(n_joints_);
  pos_max_error_ = Eigen::VectorXd::Zero(n_joints_);
  setDt(dt_in);
  setDefaultSaturation();
  setVelocityFrequencyCutOff(vel_cutoff_in);
  setPositionFrequencyCutOff(pos_cutoff_in);
  b_initialized = false;
}

JointIntegrator::~JointIntegrator() {}

void JointIntegrator::integrate(const Eigen::VectorXd acc_in,
                                const Eigen::VectorXd &vel_in,
                                const Eigen::VectorXd &pos_in,
                                Eigen::VectorXd &vel_out,
                                Eigen::VectorXd &pos_out) {
  // Use IHMC's integration scheme
  // Velocity Integration
  // Decay desired velocity to 0.0
  // default alpha_vel_=0.0124
  // default alpha_pos_=0.006
  // vel_ = (1.0 - alpha_vel_) * vel_;
  vel_ = (1.0 - alpha_vel_) * vel_ + alpha_vel_ * vel_in;
  // Integrate Joint Acceleration
  vel_ += (acc_in * dt_);
  // Clamp and Store Value
  vel_out = clampVec(vel_, vel_min_, vel_max_);
  vel_ = vel_out;

  // Position Integration
  // Decay desired position to current position
  pos_ = (1.0 - alpha_pos_) * pos_ + alpha_pos_ * pos_in;
  // Integrate Joint Position
  pos_ += (vel_ * dt_);
  // Clamp to maximum position error
  pos_ = clampVec(pos_, pos_in - pos_max_error_, pos_in + pos_max_error_);
  // Clamp to joint position limits
  pos_out = clampVec(pos_, pos_min_, pos_max_);
  // Store value
  pos_ = pos_out;
}

double JointIntegrator::getAlphaFromFrequency(const double hz,
                                              const double dt) {
  double omega = 2.0 * M_PI * hz;
  double alpha = (omega * dt) / (1.0 + (omega * dt_));
  alpha = clampValue(alpha, 0.0, 1.0);
  return alpha;
}

void JointIntegrator::initializeStates(const Eigen::VectorXd init_vel,
                                       const Eigen::VectorXd init_pos) {
  vel_ = init_vel;
  pos_ = init_pos;
  b_initialized = true;
}

void JointIntegrator::setDt(const double dt_in) { dt_ = dt_in; }
void JointIntegrator::setVelocityFrequencyCutOff(const double vel_cutoff_in) {
  vel_freq_cutoff_ = vel_cutoff_in;
  alpha_vel_ = getAlphaFromFrequency(vel_freq_cutoff_, dt_);
}
void JointIntegrator::setPositionFrequencyCutOff(const double pos_cutoff_in) {
  pos_freq_cutoff_ = pos_cutoff_in;
  alpha_pos_ = getAlphaFromFrequency(pos_freq_cutoff_, dt_);
}
void JointIntegrator::setVelocityBounds(const Eigen::VectorXd vel_min_in,
                                        const Eigen::VectorXd vel_max_in) {
  vel_min_ = vel_min_in;
  vel_max_ = vel_max_in;
}
void JointIntegrator::setPositionBounds(const Eigen::VectorXd pos_min_in,
                                        const Eigen::VectorXd pos_max_in) {
  pos_min_ = pos_min_in;
  pos_max_ = pos_max_in;
}

// Sets the maximum position deviation from current position for all joints
void JointIntegrator::setMaxPositionError(const double pos_max_error_in) {
  setMaxPositionErrorVector(default_pos_max_error_ *
                            Eigen::VectorXd::Ones(n_joints_));
}
// Use custom maximum position deviation from current position for each joint
void JointIntegrator::setMaxPositionErrorVector(
    const Eigen::VectorXd pos_max_error_in) {
  pos_max_error_ = pos_max_error_in;
}

void JointIntegrator::setDefaultSaturation() {
  setVelocityBounds(-default_vel_min_max_ * Eigen::VectorXd::Ones(n_joints_),
                    default_vel_min_max_ * Eigen::VectorXd::Ones(n_joints_));
  setPositionBounds(-default_pos_min_max_ * Eigen::VectorXd::Ones(n_joints_),
                    default_pos_min_max_ * Eigen::VectorXd::Ones(n_joints_));
  setMaxPositionError(default_pos_max_error_);
}

double JointIntegrator::clampValue(const double in, const double min,
                                   const double max) {
  if (in >= max) {
    return max;
  } else if (in <= min) {
    return min;
  } else {
    return in;
  }
}

Eigen::VectorXd JointIntegrator::clampVec(const Eigen::VectorXd vec_in,
                                          const Eigen::VectorXd vec_min,
                                          const Eigen::VectorXd vec_max) {
  Eigen::VectorXd vec_out = vec_in;
  for (int i = 0; i < vec_in.size(); i++) {
    vec_out[i] = clampValue(vec_in[i], vec_min[i], vec_max[i]);
  }
  return vec_out;
}

void JointIntegrator::printIntegrationParams() {
  std::cout << "Integration Params:" << std::endl;
  printf("  dt: %0.6f s\n", dt_);
  printf("  velocity cutoff freq: %0.3f Hz \n", vel_freq_cutoff_);
  printf("  velocity cutoff alpha: %0.6f \n", alpha_vel_);
  printf("  (1.0 - velocity cutoff alpha): %0.6f \n", (1.0 - alpha_vel_));

  printf("  position cutoff freq: %0.3f \n", pos_freq_cutoff_);
  printf("  position cutoff alpha: %0.6f \n", alpha_pos_);
  printf("  (1.0 - position cutoff alpha): %0.6f \n", (1.0 - alpha_pos_));

  // util::PrettyPrint(vel_min_, std::cout, "velocity_min");
  // util::PrettyPrint(vel_max_, std::cout, "velocity_max");
  // util::PrettyPrint(pos_min_, std::cout, "position_min");
  // util::PrettyPrint(pos_max_, std::cout, "position_max");
  // util::PrettyPrint(pos_max_error_, std::cout, "position max error");

  // util::PrettyPrint(pos_, std::cout, "position current");
  // util::PrettyPrint(vel_, std::cout, "velocity current");
}
