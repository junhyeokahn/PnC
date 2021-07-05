#include "simulator/dart/atlas/atlas_world_node.hpp"

#include "Configuration.hpp"
#include <PnC/AtlasPnC/AtlasInterface.hpp>
#include <utils/util.hpp>

AtlasWorldNode::AtlasWorldNode(const dart::simulation::WorldPtr &_world)
    : dart::gui::osg::WorldNode(_world), count_(0), t_(0.0), servo_rate_(0) {
  world_ = _world;
  robot_ = world_->getSkeleton("multisense_sl");
  n_dof_ = robot_->getNumDofs();

  interface_ = new AtlasInterface();
  sensor_data_ = new AtlasSensorData();
  command_ = new AtlasCommand();

  for (int i = 0; i < robot_->getNumJoints(); ++i) {
    dart::dynamics::Joint *joint = robot_->getJoint(i);
    if (joint->getNumDofs() == 1) {
      sensor_data_->joint_positions[joint->getName()] = 0.;
      sensor_data_->joint_velocities[joint->getName()] = 0.;
      command_->joint_positions[joint->getName()] = 0.;
      command_->joint_velocities[joint->getName()] = 0.;
      command_->joint_torques[joint->getName()] = 0.;
    }
  }

  resetButtonFlags();

  SetParams_();
}

AtlasWorldNode::~AtlasWorldNode() {
  delete interface_;
  delete sensor_data_;
  delete command_;
}

void AtlasWorldNode::customPreStep() {
  t_ = (double)count_ * servo_rate_;

  // Fill Sensor Data
  GetBaseData_(sensor_data_->base_com_pos, sensor_data_->base_com_quat,
               sensor_data_->base_com_lin_vel, sensor_data_->base_com_ang_vel,
               sensor_data_->base_joint_pos, sensor_data_->base_joint_quat,
               sensor_data_->base_joint_lin_vel,
               sensor_data_->base_joint_ang_vel);
  GetJointData_(sensor_data_->joint_positions, sensor_data_->joint_velocities);
  GetContactSwitchData_(sensor_data_->b_rf_contact, sensor_data_->b_lf_contact);

  // Check for user button presses
  if (b_button_p) {
    interface_->interrupt->b_interrupt_button_p = true;
  }
  if (b_button_r) {
    interface_->interrupt->b_interrupt_button_r = true;
  }
  if (b_button_w) {
    interface_->interrupt->b_interrupt_button_w = true;
  }
  if (b_button_a) {
    interface_->interrupt->b_interrupt_button_a = true;
  }
  if (b_button_s) {
    interface_->interrupt->b_interrupt_button_s = true;
  }
  if (b_button_d) {
    interface_->interrupt->b_interrupt_button_d = true;
  }
  if (b_button_q) {
    interface_->interrupt->b_interrupt_button_q = true;
  }
  if (b_button_e) {
    interface_->interrupt->b_interrupt_button_e = true;
  }
  if (b_button_x) {
    interface_->interrupt->b_interrupt_button_x = true;
  }
  if (b_button_j) {
    interface_->interrupt->b_interrupt_button_j = true;
  }
  if (b_button_k) {
    interface_->interrupt->b_interrupt_button_k = true;
  }

  interface_->getCommand(sensor_data_, command_);

  Eigen::VectorXd trq_cmd = Eigen::VectorXd::Zero(robot_->getNumDofs());
  trq_cmd.head(6).setZero();

  for (int i = 0; i < robot_->getNumJoints(); ++i) {
    dart::dynamics::Joint *joint = robot_->getJoint(i);
    if (joint->getNumDofs() == 1) {
      double frc = command_->joint_torques[joint->getName()] +
                   kp_ * (command_->joint_positions[joint->getName()] -
                          sensor_data_->joint_positions[joint->getName()]) +
                   kd_ * (command_->joint_velocities[joint->getName()] -
                          sensor_data_->joint_velocities[joint->getName()]);
      trq_cmd[joint->getIndexInSkeleton(0)] = frc;
    }
  }
  robot_->setForces(trq_cmd);

  count_++;

  // reset flags
  resetButtonFlags();
}

void AtlasWorldNode::GetContactSwitchData_(bool &rfoot_contact,
                                           bool &lfoot_contact) {

  double lfoot_height =
      robot_->getBodyNode("l_sole")->getTransform().translation()[2];
  double rfoot_height =
      robot_->getBodyNode("r_sole")->getTransform().translation()[2];

  if (lfoot_height < 0.005) {
    lfoot_contact = true;
  } else {
    lfoot_contact = false;
  }
  if (rfoot_height < 0.005) {
    rfoot_contact = true;
  } else {
    rfoot_contact = false;
  }
}

void AtlasWorldNode::SetParams_() {
  try {
    YAML::Node simulation_cfg =
        YAML::LoadFile(THIS_COM "Config/Atlas/dart_simulation.yaml");
    util::ReadParameter(simulation_cfg, "servo_rate", servo_rate_);
    util::ReadParameter(simulation_cfg["control_configuration"], "kp", kp_);
    util::ReadParameter(simulation_cfg["control_configuration"], "kd", kd_);
  } catch (std::runtime_error &e) {
    std::cout << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
  }
}

void AtlasWorldNode::GetBaseData_(Eigen::Vector3d &_base_com_pos,
                                  Eigen::Vector4d &_base_com_quat,
                                  Eigen::Vector3d &_base_com_lin_vel,
                                  Eigen::Vector3d &_base_com_ang_vel,
                                  Eigen::Vector3d &_base_joint_pos,
                                  Eigen::Vector4d &_base_joint_quat,
                                  Eigen::Vector3d &_base_joint_lin_vel,
                                  Eigen::Vector3d &_base_joint_ang_vel) {

  dart::dynamics::BodyNode *root_bn = robot_->getRootBodyNode();

  _base_com_pos = root_bn->getCOM();
  Eigen::Quaternion<double> base_com_quat =
      Eigen::Quaternion<double>(root_bn->getWorldTransform().linear());
  _base_com_quat << base_com_quat.w(), base_com_quat.x(), base_com_quat.y(),
      base_com_quat.z();

  _base_com_ang_vel =
      root_bn->getSpatialVelocity(root_bn->getLocalCOM()).head(3);
  _base_com_lin_vel =
      root_bn->getSpatialVelocity(root_bn->getLocalCOM()).tail(3);

  _base_joint_pos = root_bn->getCOM() - root_bn->getLocalCOM();
  _base_joint_quat = _base_com_quat;
  _base_joint_ang_vel = root_bn->getSpatialVelocity().head(3);
  _base_joint_lin_vel = root_bn->getSpatialVelocity().tail(3);
}

void AtlasWorldNode::GetJointData_(
    std::map<std::string, double> &_joint_positions,
    std::map<std::string, double> &_joint_velocities) {
  for (int i = 0; i < robot_->getNumJoints(); ++i) {
    dart::dynamics::Joint *joint = robot_->getJoint(i);
    if (joint->getNumDofs() == 1) {
      sensor_data_->joint_positions[joint->getName()] = joint->getPosition(0);
      sensor_data_->joint_velocities[joint->getName()] = joint->getVelocity(0);
    }
  }
}
