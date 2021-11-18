#include <PnC/RobotSystem/RobotSystem.hpp>
#include <PnC/A1PnC/A1Definition.hpp>
#include <PnC/A1PnC/A1Interface.hpp>
#include <PnC/A1PnC/A1StateEstimator.hpp>
#include <PnC/A1PnC/A1StateEstimator/BasicAccumulation.hpp>
#include <PnC/A1PnC/A1StateProvider.hpp>
// #include <PnC/Filter/Basic/filter.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>




AverageFilter::AverageFilter(double dt, double t_const, double limit)
      : dt_(dt), t_const_(t_const), limit_(limit) {
  myUtils::pretty_constructor(2, "Average Filter");
  est_value_ = 0.;
}

AverageFilter::~AverageFilter() {}

void AverageFilter::initialization(double _val) { est_value_ = _val; }

void AverageFilter::clear() { est_value_ = 0.; }

void AverageFilter::input(double input) {
  double update_value = input - est_value_;
  if (fabs(update_value) > limit_) {
    update_value = 0.;
  }
  est_value_ += (dt_ / (dt_ + t_const_)) * update_value;
}

double AverageFilter::output() { return est_value_; }


Eigen::VectorXd _so3_to_euler_zyx_dot(
  const Eigen::VectorXd& _global_ori_ypr,
  const Eigen::VectorXd& _global_ang_vel) {
  Eigen::MatrixXd so3_to_euler_zyx_dot_map(3, 3);
  double x(_global_ori_ypr[2]);
  double y(_global_ori_ypr[1]);
  double z(_global_ori_ypr[0]);

  if (y == M_PI / 2.0) {
    std::cout << "Singular at mapping from euler zyx to so3" << std::endl;
    exit(0);
  }

  so3_to_euler_zyx_dot_map << cos(z) * sin(y) / cos(y),
  sin(y) * sin(z) / cos(y), 1, -sin(z), cos(z), 0, cos(z) / cos(y),
  sin(z) / cos(y), 0;
  return so3_to_euler_zyx_dot_map * _global_ang_vel;
}


A1StateEstimator::A1StateEstimator(RobotSystem* robot) {
  myUtils::pretty_constructor(1, "A1 State Estimator");
  robot_ = robot;

  iso_base_com_to_imu_ = robot_->getBodyNodeIsometry(A1BodyNode::trunk).inverse() *
                         robot_->getBodyNodeIsometry(A1BodyNode::imu_link);
  iso_base_joint_to_imu_.linear() = iso_base_com_to_imu_.linear();
  iso_base_joint_to_imu_.translation() =
                         iso_base_com_to_imu_.translation() +
                         robot_->getBodyNodeIsometry(A1BodyNode::trunk).linear() *
                         robot_->getBaseLocalCOMPos();
  // .linear() is the 3x3 rot matrix
  // .translation() is a 3D vector/location

  sp_ = A1StateProvider::getStateProvider(robot_);

  curr_config_ = Eigen::VectorXd::Zero(robot_->getNumDofs());
  prev_config_ = Eigen::VectorXd::Zero(robot_->getNumDofs());

  global_linear_offset_.setZero();

  curr_qdot_ = Eigen::VectorXd::Zero(robot_->getNumDofs());
  prev_qdot_ = curr_qdot_;

  global_body_euler_zyx_.setZero();
  global_body_quat_ = Eigen::Quaternion<double>::Identity();

  global_body_euler_zyx_dot_.setZero();
  prev_body_euler_zyx_dot_ = global_body_euler_zyx_dot_;

  virtual_q_ = Eigen::VectorXd::Zero(6);
  virtual_qdot_ = Eigen::VectorXd::Zero(6);

  joint_velocity_filter_freq_ = 100.0;    // Hz
  angular_velocity_filter_freq_ = 100.0;  // Hz

  // TODO: Find t_const and limit vals
  x_vel_est_ = new AverageFilter(A1Aux::servo_rate, 0.030, 0.15);
  y_vel_est_ = new AverageFilter(A1Aux::servo_rate, 0.030, 0.4);
  z_vel_est_ = new AverageFilter(A1Aux::servo_rate, 0.030, 0.2);

}


A1StateEstimator::~A1StateEstimator() {
  delete x_vel_est_;
  delete y_vel_est_;
  delete z_vel_est_;
}

void A1StateEstimator::Initialization(A1SensorData* data) {
  _COMAngularUpdate(data);
  _JointUpdate(data);
  _ConfigurationAndModelUpdate(data);
  sp_->jpos_ini = curr_config_.segment(A1::n_vdof, A1::n_adof);

  x_vel_est_->initialization(sp_->com_vel[0]);
  y_vel_est_->initialization(sp_->com_vel[1]);
  z_vel_est_->initialization(sp_->com_vel[2]);

  _FootContactUpdate(data);
  sp_->saveCurrentData();
}

void A1StateEstimator::_COMAngularUpdate(A1SensorData* data) {

  // - convert imu rpy to R_rpy
  Eigen::Quaternion<double> rpy_quat = myUtils::EulerZYXtoQuat(data->imu_rpy[0], data->imu_rpy[1], data->imu_rpy[2]);
  Eigen::MatrixXd R_rpy = rpy_quat.toRotationMatrix();
  // myUtils::pretty_print(R_rpy, std::cout, "R_rpy");
  // - define local transform from root joint to imu R_(in constructor, it is a const) --> iso_base_joint_to_imu_.linear()
  // - R_rpy * R_.inverse()
  Eigen::Matrix3d R_world_robot = R_rpy * iso_base_joint_to_imu_.linear().inverse();

  // - result is a rotation matrix from world to root joint R_world_root
  // - convert this matrix to ZYX again and put in global ... _zyx
  Eigen::Quaternion<double> quat_world_robot  = Eigen::Quaternion<double>(R_world_robot);
  global_body_euler_zyx_ = myUtils::QuatToEulerZYX(quat_world_robot); // yaw, pitch, roll

//////////////////////////////////////////////////////////////////////////////////
  Eigen::MatrixXd tmp_ryan_todo = R_world_robot;
  // myUtils::pretty_print(R_world_robot, std::cout, "R world frame to robot torso frame");
  // myUtils::pretty_print(tmp_ryan_todo, std::cout, "R_world_robot");
  Eigen::MatrixXd tmp_ = robot_->getBodyNodeIsometry(A1BodyNode::trunk).linear();
  // myUtils::pretty_print(tmp_, std::cout, "Sim R_world robot");


  Eigen::VectorXd tmp_body_ang_vel = robot_->getBodyNodeSpatialVelocity(A1BodyNode::trunk).head(3);

  double r, p, y; r = data->virtual_q[3]; p = data->virtual_q[4]; y = data->virtual_q[5];
  Eigen::MatrixXd sim_R_wr;
  // sim_R_wr << 
//////////////////////////////////////////////////////////////////////////////////

  // - local_imu = R_ * imu_ang_vel
  // - result is the local ang vel to root joint
  Eigen::Vector3d local_robot_root_ang_vel_ = iso_base_joint_to_imu_.linear() * data->imu_ang_vel;
  // - R_world_root * local_imu -> global angular velocity
  Eigen::Vector3d global_ang_vel_ = R_world_robot * local_robot_root_ang_vel_;
  // myUtils::pretty_print(tmp_body_ang_vel, std::cout, "sim global ang vel");
  // myUtils::pretty_print(global_ang_vel_, std::cout, "global_ang_vel_");

// - convert to euler zyx from there
  global_body_euler_zyx_dot_ = _so3_to_euler_zyx_dot(global_body_euler_zyx_, global_ang_vel_);

  global_body_quat_ = Eigen::Quaternion<double> (
        dart::math::eulerZYXToMatrix(global_body_euler_zyx_));

  curr_config_[3] = data->virtual_q[3];
  curr_config_[4] = data->virtual_q[4];
  curr_config_[5] = data->virtual_q[5];
  curr_qdot_[3] = data->virtual_qdot[3];
  curr_qdot_[4] = data->virtual_qdot[4];
  curr_qdot_[5] = data->virtual_qdot[5];
// Update curr_config [3,4,5], curr_qdot[3,4,5]
  curr_config_[3] = global_body_euler_zyx_[0];
  curr_config_[4] = global_body_euler_zyx_[1];
  curr_config_[5] = global_body_euler_zyx_[2];

  // myUtils::pretty_print(curr_config_, std::cout, "current configuration");
  // myUtils::pretty_print(data->virtual_q, std::cout, "virtual q");

  curr_qdot_[3] = global_body_euler_zyx_dot_[0];
  curr_qdot_[4] = global_body_euler_zyx_dot_[1];
  curr_qdot_[5] = global_body_euler_zyx_dot_[2];

  // myUtils::pretty_print(curr_qdot_, std::cout, "current config dot");
  // myUtils::pretty_print(data->virtual_qdot, std::cout, "virtual q dot");

}

void A1StateEstimator::Update(A1SensorData* data) {
  _JointUpdate(data);

  _COMAngularUpdate(data);

  _ConfigurationAndModelUpdate(data);

  // TODO: What values are in sp_->com_vel / Who sets them?
  x_vel_est_->input(sp_->com_vel[0]);
  sp_->est_com_vel[0] = x_vel_est_->output();
  // sp_->est_com_vel[0] = data->virtual_qdot[0];

  y_vel_est_->input(sp_->com_vel[1]);
  sp_->est_com_vel[1] = y_vel_est_->output();
  // sp_->est_com_vel[1] = data->virtual_qdot[1];

  z_vel_est_->input(sp_->com_vel[2]);
  sp_->est_com_vel[2] = z_vel_est_->output();
  // sp_->est_com_vel[2] = data->virtual_qdot[2];


  /*myUtils::pretty_print(sp_->est_com_vel, std::cout, "estimated com vel");
  Eigen::VectorXd tmp = data->virtual_qdot;
  Eigen::VectorXd tmp2 = data->virtual_q;
  myUtils::pretty_print(tmp2, std::cout, "Sim com pos");
  myUtils::pretty_print(tmp, std::cout, "Sim com vel");*/
  // std::cout << "-----------------------------------------------------" << std::endl;

  _FootContactUpdate(data);

  sp_->frfoot_pos = robot_->getBodyNodeIsometry(A1BodyNode::FR_foot).translation();
  sp_->flfoot_pos = robot_->getBodyNodeIsometry(A1BodyNode::FL_foot).translation(); 
  sp_->rrfoot_pos = robot_->getBodyNodeIsometry(A1BodyNode::RR_foot).translation(); 
  sp_->rlfoot_pos = robot_->getBodyNodeIsometry(A1BodyNode::RL_foot).translation();
  sp_->frfoot_vel = robot_->getBodyNodeSpatialVelocity(A1BodyNode::FR_foot).tail(3);
  sp_->flfoot_vel = robot_->getBodyNodeSpatialVelocity(A1BodyNode::FL_foot).tail(3);
  sp_->rrfoot_vel = robot_->getBodyNodeSpatialVelocity(A1BodyNode::RR_foot).tail(3);
  sp_->rlfoot_vel = robot_->getBodyNodeSpatialVelocity(A1BodyNode::RL_foot).tail(3);
  sp_->imu_ang_vel = data->imu_ang_vel;
  sp_->imu_acc = data->imu_acc;
  sp_->imu_rpy = data->imu_rpy;

  sp_->saveCurrentData();
}

void A1StateEstimator::_JointUpdate(A1SensorData* data) {
  curr_config_.setZero();
  curr_qdot_.setZero();

  // TODO: Remove once we implement COMAngular
  // These are the vals we can compare with
  // But remember there may be offset because our estimator considers the foot at 0,0,0 instead of the torso
  // for (int i = 0; i < A1::n_vdof; ++i) {
  //   curr_config_[i] = data->virtual_q[i];
  //   curr_qdot_[i] = data->virtual_qdot[i];
  // }
  for (int i(0); i < A1::n_adof; ++i) {
    curr_config_[A1::n_vdof + i] = data->q[i];
    curr_qdot_[A1::n_vdof + i] = data->qdot[i];
  }

}

void A1StateEstimator::_ConfigurationAndModelUpdate(A1SensorData* data) {

  robot_->updateSystem(curr_config_, curr_qdot_, false); // update robot as is correct ori but 0,0,0 floating base
  Eigen::VectorXd foot_pos;
  Eigen::VectorXd foot_vel;
  if (sp_->front_stance_foot == A1BodyNode::FR_foot) {
    foot_pos =
        robot_->getBodyNodeIsometry(A1BodyNode::FR_foot).translation();
    foot_vel =
        robot_->getBodyNodeSpatialVelocity(A1BodyNode::FR_foot).tail(3);
  } else {
    foot_pos =
        robot_->getBodyNodeIsometry(A1BodyNode::FL_foot).translation();
    foot_vel =
        robot_->getBodyNodeSpatialVelocity(A1BodyNode::FL_foot).tail(3);
  }// Foot pos will be under the ground

  // check if stance foot changes. If so, find the new linear offset
  if (sp_->front_stance_foot != sp_->prev_front_stance_foot) {
    Eigen::Vector3d new_stance_foot = foot_pos;
    Eigen::Vector3d old_stance_foot;
    if (sp_->prev_front_stance_foot == A1BodyNode::FR_foot) {
      old_stance_foot =
          robot_->getBodyNodeIsometry(A1BodyNode::FR_foot).translation();
    } else {
      old_stance_foot =
          robot_->getBodyNodeIsometry(A1BodyNode::FL_foot).translation();
    }

    Eigen::Vector3d old_to_new = new_stance_foot - old_stance_foot;
    global_linear_offset_ += old_to_new;
    // myUtils::pretty_print(new_stance_foot, std::cout, "new_stance_foot");
    // myUtils::pretty_print(old_stance_foot, std::cout, "old_stance_foot");
    // myUtils::pretty_print(stance_difference, std::cout, "stance_difference");
    // myUtils::pretty_print(old_estimate, std::cout, "old_estimate");
    // myUtils::pretty_print(new_estimate, std::cout, "new_estimate");
    // myUtils::pretty_print(offset_update, std::cout, "offset_update");

  }
  // Perform Base update using kinematics
  curr_config_[0] = global_linear_offset_[0] - foot_pos[0];// pushing the base upwards to put our contact feet on the ground
  curr_config_[1] = global_linear_offset_[1] - foot_pos[1];// i.e contact foot position is on ground
  curr_config_[2] = global_linear_offset_[2] - foot_pos[2];

  // Update qdot using the difference between the curr_config_ now and previous
  curr_qdot_.head(3) =
      (curr_config_.head(3) - prev_config_.head(3)) / (A1Aux::servo_rate);

  robot_->updateSystem(curr_config_, curr_qdot_, false);// Again call updateSystem to include linear and orientation 
  sp_->q = curr_config_;
  sp_->qdot = curr_qdot_;
  sp_->com_pos = robot_->getCoMPosition();
  sp_->com_vel = robot_->getCoMVelocity();

  // myUtils::pretty_print(sp_->com_pos, std::cout, "sp_->com_pos should = kinematics pos");
  // myUtils::pretty_print(sp_->com_vel, std::cout, "sp_->com_vel should = kinematics vel");

  // update previous stance foot.
  sp_->prev_front_stance_foot = sp_->front_stance_foot;
  // update previous config:
  prev_config_ = curr_config_;


  Eigen::VectorXd body_pos_kinematics = curr_config_.head(3);
  Eigen::VectorXd body_vel_kinematics = curr_qdot_.head(3);
  // myUtils::pretty_print(body_pos_kinematics, std::cout, "body_pos_kinematics");
  // myUtils::pretty_print(body_vel_kinematics, std::cout, "body_vel_kinematics");
}

void A1StateEstimator::_FootContactUpdate(A1SensorData* data) {
  sp_->foot_force = data->foot_force;

  // TODO: New contact method
  if (data->foot_force[0] >= -5) sp_->b_frfoot_contact = 1;
  else sp_->b_frfoot_contact = 0;
  if (data->foot_force[1] >= -5) sp_->b_flfoot_contact = 1;
  else sp_->b_flfoot_contact = 0;
  if (data->foot_force[0] >= -1) sp_->b_rrfoot_contact = 1;
  else sp_->b_rrfoot_contact = 0;
  if (data->foot_force[0] >= -1) sp_->b_rlfoot_contact = 1;
  else sp_->b_rlfoot_contact = 0;
}




double A1StateEstimator::clamp_value(double in, double min, double max) {
  if (in >= max) return max;
  else if (in <= min) return min;
  else return in;
}

double A1StateEstimator::computeAlphaGivenBreakFrequency(double hz,
                                                         double dt) {
  double omega = 2. * M_PI * hz;
  double alpha = (omega * dt) / (1. + (omega * dt));
  alpha = clamp_value(alpha, 0.0, 1.0);
  return alpha;
}



