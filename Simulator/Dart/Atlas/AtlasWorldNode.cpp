#include <Configuration.h>
#include <PnC/AtlasPnC/AtlasInterface.hpp>
#include <Simulator/Dart/Atlas/AtlasWorldNode.hpp>
#include <Utils/IO/DataManager.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>

AtlasWorldNode::AtlasWorldNode(const dart::simulation::WorldPtr &_world)
    : dart::gui::osg::WorldNode(_world), count_(0), t_(0.0), servo_rate_(0) {
  world_ = _world;
  robot_ = world_->getSkeleton("multisense_sl");
  trq_lb_ = robot_->getForceLowerLimits();
  trq_ub_ = robot_->getForceUpperLimits();
  n_dof_ = robot_->getNumDofs();

  trq_cmd_ = Eigen::VectorXd::Zero(n_dof_);

  interface_ = new AtlasInterface();
  sensor_data_ = new AtlasSensorData();
  command_ = new AtlasCommand();

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

  sensor_data_->q = robot_->getPositions().tail(n_dof_ - 6);
  sensor_data_->virtual_q = robot_->getPositions().head(6);
  sensor_data_->qdot = robot_->getVelocities().tail(n_dof_ - 6);
  sensor_data_->virtual_qdot = robot_->getVelocities().head(6);

  // Use force thresholding to detect contacts
  GetContactSwitchData_(sensor_data_->rfoot_contact,
                        sensor_data_->lfoot_contact);

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

  trq_cmd_.tail(n_dof_ - 6) = command_->jtrq;
  for (int i = 0; i < n_dof_ - 6; ++i) {
    trq_cmd_[i + 6] += kp_ * (command_->q[i] - sensor_data_->q[i]) +
                       kd_ * (command_->qdot[i] - sensor_data_->qdot[i]);
  }

  // Crop Torque to be within the limits
  trq_cmd_.tail(n_dof_ - 6) = myUtils::CropVector(
      trq_cmd_.tail(n_dof_ - 6), trq_lb_.segment(Atlas::n_vdof, Atlas::n_adof),
      trq_ub_.segment(Atlas::n_vdof, Atlas::n_adof), "clip trq in sim");

  trq_cmd_.head(6).setZero();

  robot_->setForces(trq_cmd_);

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
        YAML::LoadFile(THIS_COM "Config/Atlas/SIMULATION.yaml");
    myUtils::readParameter(simulation_cfg, "servo_rate", servo_rate_);
    myUtils::readParameter(simulation_cfg["control_configuration"], "kp", kp_);
    myUtils::readParameter(simulation_cfg["control_configuration"], "kd", kd_);
  } catch (std::runtime_error &e) {
    std::cout << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
  }
}
