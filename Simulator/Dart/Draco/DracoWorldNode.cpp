#include <Configuration.h>
#include <PnC/DracoPnC/DracoInterface.hpp>
#include <Simulator/Dart/Draco/DracoWorldNode.hpp>
#include <Utils/IO/DataManager.hpp>
#include <Utils/IO/IOUtilities.hpp>

DracoWorldNode::DracoWorldNode(const dart::simulation::WorldPtr& _world)
    : dart::gui::osg::WorldNode(_world),
      count_(0),
      t_(0.0),
      servo_rate_(0.001) {
  world_ = _world;
  skel_ = world_->getSkeleton("Draco");
  trq_lb_ = skel_->getForceLowerLimits();
  trq_ub_ = skel_->getForceUpperLimits();
  ground_ = world_->getSkeleton("ground_skeleton");
  dof_ = skel_->getNumDofs();
  trq_cmd_ = Eigen::VectorXd::Zero(dof_);

  interface_ = new DracoInterface();
  sensor_data_ = new DracoSensorData();
  command_ = new DracoCommand();

  resetButtonFlags();
  set_parameters_();
}

DracoWorldNode::~DracoWorldNode() {
  delete interface_;
  delete sensor_data_;
  delete command_;
}

void DracoWorldNode::set_parameters_() {
  try {
    YAML::Node simulation_cfg =
        YAML::LoadFile(THIS_COM "Config/Draco/SIMULATION.yaml");
    myUtils::readParameter(simulation_cfg, "servo_rate", servo_rate_);
    myUtils::readParameter(simulation_cfg, "release_time", release_time_);
    YAML::Node control_cfg = simulation_cfg["control_configuration"];
    myUtils::readParameter(control_cfg, "kp", kp_);
    myUtils::readParameter(control_cfg, "kd", kd_);
  } catch (std::runtime_error& e) {
    std::cout << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
  }
}

void DracoWorldNode::customPreStep() {
  t_ = (double)count_ * servo_rate_;

  sensor_data_->q = skel_->getPositions().tail(10);
  sensor_data_->qdot = skel_->getVelocities().tail(10);
  sensor_data_->jtrq = skel_->getForces().tail(10);

  get_force_torque_data_();
  get_imu_data_(sensor_data_->imu_ang_vel, sensor_data_->imu_acc,
                sensor_data_->imu_mag);
  check_foot_contact_by_pos_(sensor_data_->rfoot_contact,
                             sensor_data_->lfoot_contact);
  // check_foot_contact_by_ft_(sensor_data_->rfoot_contact,
  // sensor_data_->lfoot_contact);

  // Check for user button presses
  if (b_button_p) interface_->interrupt->b_interrupt_button_p = true;
  if (b_button_r) interface_->interrupt->b_interrupt_button_r = true;
  if (b_button_w) interface_->interrupt->b_interrupt_button_w = true;
  if (b_button_a) interface_->interrupt->b_interrupt_button_a = true;
  if (b_button_s) interface_->interrupt->b_interrupt_button_s = true;
  if (b_button_d) interface_->interrupt->b_interrupt_button_d = true;
  if (b_button_q) interface_->interrupt->b_interrupt_button_q = true;
  if (b_button_e) interface_->interrupt->b_interrupt_button_e = true;
  if (b_button_x) interface_->interrupt->b_interrupt_button_x = true;
  if (b_button_j) interface_->interrupt->b_interrupt_button_j = true;
  if (b_button_k) interface_->interrupt->b_interrupt_button_k = true;
  if (b_button_h) interface_->interrupt->b_interrupt_button_h = true;
  if (b_button_l) interface_->interrupt->b_interrupt_button_l = true;

  interface_->getCommand(sensor_data_, command_);

  trq_cmd_.setZero();

  // Low level FeedForward and Position Control
  trq_cmd_.tail(10) = command_->jtrq;
  // myUtils::pretty_print(trq_cmd_, std::cout, "ff_torques");
  for (int i = 0; i < 10; ++i) {
    trq_cmd_[i + 6] += kp_[i] * (command_->q[i] - sensor_data_->q[i]) +
                       kd_[i] * (command_->qdot[i] - sensor_data_->qdot[i]);
  }
  trq_cmd_.head(6).setZero();

  // hold robot at the initial phase
  if (t_ < release_time_) {
    hold_xy_();
    hold_rot_();
  } else {
    static bool first__ = true;
    if (first__) {
      std::cout << "[Release]" << std::endl;
      first__ = false;
    }
  }

  // std::cout << "q" << std::endl;
  // std::cout << skel_->getPositions().transpose() << std::endl;

  // myUtils::pretty_print(trq_cmd_, std::cout, "torques");

  skel_->setForces(trq_cmd_);

  count_++;

  // reset flags
  resetButtonFlags();
}

void DracoWorldNode::get_imu_data_(Eigen::VectorXd& ang_vel,
                                   Eigen::VectorXd& acc,
                                   Eigen::VectorXd& imu_mag) {
  // angvel
  Eigen::VectorXd ang_vel_local =
      skel_->getBodyNode("IMU")
          ->getSpatialVelocity(dart::dynamics::Frame::World(),
                               skel_->getBodyNode("IMU"))
          .head(3);

  ang_vel = ang_vel_local;
  Eigen::MatrixXd R_world_imu(3, 3);
  R_world_imu = skel_->getBodyNode("IMU")->getWorldTransform().linear();
  // acc
  Eigen::Vector3d linear_imu_acc =
      skel_->getBodyNode("IMU")->getCOMLinearAcceleration();
  Eigen::Vector3d global_grav(0, 0, 9.81);
  // acc = R_world_imu.transpose() * (global_grav + linear_imu_acc);
  acc = R_world_imu.transpose() * (global_grav);

  // mag
  Eigen::VectorXd global_mag = Eigen::VectorXd::Zero(3);
  global_mag[0] = 1.;
  imu_mag = R_world_imu.transpose() * global_mag;
}

void DracoWorldNode::check_foot_contact_by_ft_(bool& rfoot_contact,
                                               bool& lfoot_contact) {
  // Get Sensor Wrench Data
  Eigen::VectorXd rf_wrench = sensor_data_->rf_wrench;
  Eigen::VectorXd lf_wrench = sensor_data_->lf_wrench;

  // Local Z-Force Threshold
  double force_threshold = 10;  // 10 Newtons ~ 1kg. If sensor detects this
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
}

void DracoWorldNode::check_foot_contact_by_pos_(bool& rfoot_contact,
                                                bool& lfoot_contact) {
  Eigen::VectorXd r_c = skel_->getBodyNode("rFootCenter")->getCOM();
  Eigen::VectorXd l_c = skel_->getBodyNode("lFootCenter")->getCOM();
  Eigen::VectorXd r_f = skel_->getBodyNode("rFootFront")->getCOM();
  Eigen::VectorXd l_f = skel_->getBodyNode("lFootFront")->getCOM();
  Eigen::VectorXd r_b = skel_->getBodyNode("rFootBack")->getCOM();
  Eigen::VectorXd l_b = skel_->getBodyNode("lFootBack")->getCOM();

  if ((fabs(l_c[2]) < 0.002) || (fabs(l_f[2]) < 0.002) ||
      (fabs(l_b[2] < 0.002))) {
    lfoot_contact = true;
    // printf("left contact\n");
  } else {
    lfoot_contact = false;
  }

  if ((fabs(r_c[2]) < 0.002) || (fabs(r_f[2]) < 0.002) ||
      (fabs(r_b[2] < 0.002))) {
    rfoot_contact = true;
    // printf("right contact\n");
  } else {
    rfoot_contact = false;
  }
}

void DracoWorldNode::hold_rot_() {
  Eigen::VectorXd q = skel_->getPositions();
  Eigen::VectorXd v = skel_->getVelocities();
  double kp(200);
  double kd(5);
  trq_cmd_[3] = kp * (-q[3]) + kd * (-v[3]);
  trq_cmd_[4] = kp * (-q[4]) + kd * (-v[4]);
  trq_cmd_[5] = kp * (-q[5]) + kd * (-v[5]);
}

void DracoWorldNode::hold_xy_() {
  static double des_x = (skel_->getPositions())[0];
  static double des_y = (skel_->getPositions())[1];
  static double des_z = (skel_->getPositions())[2];
  static double des_xdot(0.);
  static double des_ydot(0.);
  static double des_zdot(0.);

  Eigen::VectorXd q = skel_->getPositions();
  Eigen::VectorXd v = skel_->getVelocities();

  double kp(500);
  double kd(100);

  trq_cmd_[0] = kp * (des_x - q[0]) + kd * (des_xdot - v[0]);
  trq_cmd_[1] = kp * (des_y - q[1]) + kd * (des_ydot - v[1]);
  // trq_cmd_[2] = kp * (des_z - q[2]) + kd * (des_zdot - v[2]);
}

void DracoWorldNode::get_force_torque_data_() {
  Eigen::VectorXd rf_wrench = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd lf_wrench = Eigen::VectorXd::Zero(6);

  dart::dynamics::BodyNode* lfoot_bn = skel_->getBodyNode("lAnkle");
  dart::dynamics::BodyNode* rfoot_bn = skel_->getBodyNode("rAnkle");
  const dart::collision::CollisionResult& _result =
      world_->getLastCollisionResult();

  for (const auto& contact : _result.getContacts()) {
    for (const auto& shapeNode :
         lfoot_bn->getShapeNodesWith<dart::dynamics::CollisionAspect>()) {
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
        Eigen::Isometry3d T_wc = Eigen::Isometry3d::Identity();
        T_wc.translation() = contact.point;
        Eigen::Isometry3d T_wa = skel_->getBodyNode("lAnkle")->getTransform(
            dart::dynamics::Frame::World());
        Eigen::Isometry3d T_ca = T_wc.inverse() * T_wa;
        Eigen::MatrixXd AdT_ca = dart::math::getAdTMatrix(T_ca);
        Eigen::VectorXd w_a = Eigen::VectorXd::Zero(6);
        w_a = AdT_ca.transpose() * w_c;
        lf_wrench += w_a;
      }
    }
    for (const auto& shapeNode :
         rfoot_bn->getShapeNodesWith<dart::dynamics::CollisionAspect>()) {
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
        Eigen::Isometry3d T_wa = skel_->getBodyNode("rAnkle")->getTransform(
            dart::dynamics::Frame::World());
        Eigen::Isometry3d T_ca = T_wc.inverse() * T_wa;
        Eigen::MatrixXd AdT_ca = dart::math::getAdTMatrix(T_ca);
        Eigen::VectorXd w_a = Eigen::VectorXd::Zero(6);
        w_a = AdT_ca.transpose() * w_c;
        rf_wrench += w_a;
      }
    }
  }

  sensor_data_->lf_wrench = lf_wrench;
  sensor_data_->rf_wrench = rf_wrench;
}
