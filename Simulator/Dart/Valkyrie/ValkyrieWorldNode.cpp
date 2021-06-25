#include <Configuration.hpp>
#include <PnC/ValkyriePnC/ValkyrieInterface.hpp>
#include <Simulator/Dart/Valkyrie/ValkyrieWorldNode.hpp>
#include <Utils/IO/DataManager.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>

ValkyrieWorldNode::ValkyrieWorldNode(const dart::simulation::WorldPtr &_world)
    : dart::gui::osg::WorldNode(_world), count_(0), t_(0.0), servo_rate_(0) {
  world_ = _world;
  robot_ = world_->getSkeleton("valkyrie");
  trq_lb_ = robot_->getForceLowerLimits();
  trq_ub_ = robot_->getForceUpperLimits();
  n_dof_ = robot_->getNumDofs();
  ground_ = world_->getSkeleton("ground_skeleton");

  trq_cmd_ = Eigen::VectorXd::Zero(n_dof_);

  interface_ = new ValkyrieInterface();
  sensor_data_ = new ValkyrieSensorData();
  command_ = new ValkyrieCommand();

  resetButtonFlags();

  SetParams_();
}

ValkyrieWorldNode::~ValkyrieWorldNode() {
  delete interface_;
  delete sensor_data_;
  delete command_;
}

void ValkyrieWorldNode::customPreStep() {
  t_ = (double)count_ * servo_rate_;

  sensor_data_->q = robot_->getPositions().tail(n_dof_ - 6);
  sensor_data_->virtual_q = robot_->getPositions().head(6);
  sensor_data_->qdot = robot_->getVelocities().tail(n_dof_ - 6);
  sensor_data_->virtual_qdot = robot_->getVelocities().head(6);

  // Compute local frame wrenches on the sensor
  GetForceTorqueData_();
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
      trq_cmd_.tail(n_dof_ - 6),
      trq_lb_.segment(Valkyrie::n_vdof, Valkyrie::n_adof),
      trq_ub_.segment(Valkyrie::n_vdof, Valkyrie::n_adof), "clip trq in sim");

  trq_cmd_.head(6).setZero();

  robot_->setForces(trq_cmd_);

  count_++;

  // reset flags
  resetButtonFlags();
}

void ValkyrieWorldNode::GetContactSwitchData_(bool &rfoot_contact,
                                              bool &lfoot_contact) {
  // Get Sensor Wrench Data
  Eigen::VectorXd rf_wrench = sensor_data_->rf_wrench;
  Eigen::VectorXd lf_wrench = sensor_data_->lf_wrench;

  // Local Z-Force Threshold
  double force_threshold = 10; // 10 Newtons ~ 1kg. If sensor detects this
                               // force, then we are in contact

  if (fabs(rf_wrench[5]) >= force_threshold) {
    rfoot_contact = true;
  } else {
    rfoot_contact = false;
  }

  if (fabs(lf_wrench[5]) >= force_threshold) {
    lfoot_contact = true;
  } else {
    lfoot_contact = false;
  }

  // std::cout << "rwrench = " << fabs(rf_wrench[5]) << std::endl;
  // std::cout << "lwrench = " << fabs(lf_wrench[5]) << std::endl;
  // std::cout << "Rfoot contact = " << rfoot_contact << std::endl;
  // std::cout << "Lfoot contact = " << lfoot_contact << std::endl;
}

void ValkyrieWorldNode::SetParams_() {
  try {
    YAML::Node simulation_cfg =
        YAML::LoadFile(THIS_COM "Config/Valkyrie/SIMULATION.yaml");
    myUtils::readParameter(simulation_cfg, "servo_rate", servo_rate_);
    myUtils::readParameter(simulation_cfg["control_configuration"], "kp", kp_);
    myUtils::readParameter(simulation_cfg["control_configuration"], "kd", kd_);

  } catch (std::runtime_error &e) {
    std::cout << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
  }
}

void ValkyrieWorldNode::GetForceTorqueData_() {
  Eigen::VectorXd rf_wrench = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd lf_wrench = Eigen::VectorXd::Zero(6);

  dart::dynamics::BodyNode *lfoot_bn = robot_->getBodyNode("leftFoot");
  dart::dynamics::BodyNode *rfoot_bn = robot_->getBodyNode("rightFoot");
  const dart::collision::CollisionResult &_result =
      world_->getLastCollisionResult();

  Eigen::VectorXd lf_contact_force_sum = Eigen::VectorXd::Zero(3);
  for (const auto &contact : _result.getContacts()) {
    for (const auto &shapeNode :
         lfoot_bn->getShapeNodesWith<dart::dynamics::CollisionAspect>()) {
      // Ensure that we view the force as external.
      double sgn = 1.0;
      if (shapeNode == contact.collisionObject1->getShapeFrame()) {
        sgn = 1.0;
      }
      if (shapeNode == contact.collisionObject2->getShapeFrame()) {
        sgn = -1.0;
      }
      // Perform Adjoint Map to local frame wrench
      if (shapeNode == contact.collisionObject1->getShapeFrame() ||
          shapeNode == contact.collisionObject2->getShapeFrame()) {
        Eigen::VectorXd w_c = Eigen::VectorXd::Zero(6);
        w_c.tail(3) = (contact.force * sgn);
        lf_contact_force_sum += (contact.force * sgn);
        Eigen::Isometry3d T_wc = Eigen::Isometry3d::Identity();
        T_wc.translation() = contact.point;
        Eigen::Isometry3d T_wa =
            robot_->getBodyNode("leftFoot")
                ->getTransform(dart::dynamics::Frame::World());
        Eigen::Isometry3d T_ca = T_wc.inverse() * T_wa;
        Eigen::MatrixXd AdT_ca = dart::math::getAdTMatrix(T_ca);
        Eigen::VectorXd w_a = Eigen::VectorXd::Zero(6);
        w_a = AdT_ca.transpose() * w_c;
        lf_wrench += w_a;
      }
    }
    for (const auto &shapeNode :
         rfoot_bn->getShapeNodesWith<dart::dynamics::CollisionAspect>()) {
      // Conditional Check to ensure that we view the force as external.
      double sgn = 1.0;
      if (shapeNode == contact.collisionObject1->getShapeFrame()) {
        sgn = 1.0;
      }
      if (shapeNode == contact.collisionObject2->getShapeFrame()) {
        sgn = -1.0;
      }
      // Perform Adjoint Map to local frame wrench
      if (shapeNode == contact.collisionObject1->getShapeFrame() ||
          shapeNode == contact.collisionObject2->getShapeFrame()) {
        double normal(contact.normal(2));
        Eigen::VectorXd w_c = Eigen::VectorXd::Zero(6);
        w_c.tail(3) = (contact.force * sgn);
        Eigen::Isometry3d T_wc = Eigen::Isometry3d::Identity();
        T_wc.translation() = contact.point;
        Eigen::Isometry3d T_wa =
            robot_->getBodyNode("rightFoot")
                ->getTransform(dart::dynamics::Frame::World());
        Eigen::Isometry3d T_ca = T_wc.inverse() * T_wa;
        Eigen::MatrixXd AdT_ca = dart::math::getAdTMatrix(T_ca);
        Eigen::VectorXd w_a = Eigen::VectorXd::Zero(6);
        w_a = AdT_ca.transpose() * w_c;
        // myUtils::pretty_print(w_a, std::cout, "right");
        rf_wrench += w_a;
      }
    }
  }

  // myUtils::pretty_print(lf_contact_force_sum, std::cout,
  // "lf_contact_force_sum");
  // myUtils::pretty_print(rf_wrench, std::cout, "sensor true local rf_wrench");
  // myUtils::pretty_print(lf_wrench, std::cout, "sensor true local lf_wrench
  // ");

  sensor_data_->lf_wrench = lf_wrench;
  sensor_data_->rf_wrench = rf_wrench;
}
