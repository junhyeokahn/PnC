#include <Configuration.h>
#include <PnC/A1PnC/A1Interface.hpp>
#include <Simulator/Dart/A1/A1WorldNode.hpp>
#include <Utils/IO/DataManager.hpp>
#include <Utils/IO/IOUtilities.hpp>

A1WorldNode::A1WorldNode(const dart::simulation::WorldPtr& _world)
    : dart::gui::osg::WorldNode(_world),
      count_(0),
      t_(0.0),
      servo_rate_(0.001) {
  world_ = _world;
  skel_ = world_->getSkeleton("a1");
  trq_lb_ = skel_->getForceLowerLimits();
  trq_ub_ = skel_->getForceUpperLimits();
  ground_ = world_->getSkeleton("ground_skeleton");
  dof_ = skel_->getNumDofs();
  trq_cmd_ = Eigen::VectorXd::Zero(dof_);

  // interface_ = new A1Interface();
  sensor_data_ = new A1SensorData();
  command_ = new A1Command();

  resetButtonFlags();
  set_parameters_();
}

A1WorldNode::~A1WorldNode() {
  delete interface_;
  delete sensor_data_;
  delete command_;
}

void A1WorldNode::set_parameters_() {
  try {
    YAML::Node simulation_cfg =
        YAML::LoadFile(THIS_COM "Config/A1/SIMULATION.yaml");
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

void A1WorldNode::customPreStep() {
  t_ = (double)count_ * servo_rate_;

  sensor_data_->q = skel_->getPositions().tail(12);
  sensor_data_->qdot = skel_->getVelocities().tail(12);
  sensor_data_->jtrq = skel_->getForces().tail(12);

  // get_force_torque_data_(); // TODO
  get_imu_data_(sensor_data_->imu_ang_vel, sensor_data_->imu_acc);
  check_foot_contact_by_pos_(sensor_data_->frfoot_contact,
                             sensor_data_->flfoot_contact,
                             sensor_data_->rrfoot_contact,
                             sensor_data_->rlfoot_contact);
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
  trq_cmd_.tail(12) = command_->jtrq;
  // myUtils::pretty_print(trq_cmd_, std::cout, "ff_torques");
  for (int i = 0; i < 12; ++i) {
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
  //

  // TEST
  // if (t_ > 6. && t_ < 10.) {
  // std::cout << "giving dist" << std::endl;
  // trq_cmd_[1] = -10.;
  //}
  // TEST

  skel_->setForces(trq_cmd_);

  count_++;

  // reset flags
  resetButtonFlags();
}

void A1WorldNode::get_imu_data_(Eigen::VectorXd& ang_vel,
                                   Eigen::VectorXd& acc) {
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

}

void A1WorldNode::check_foot_contact_by_ft_(bool& frfoot_contact,
                                            bool& flfoot_contact,
                                            bool& rrfoot_contact,
                                            bool& rlfoot_contact) {
  // Get Sensor Wrench Data
  Eigen::VectorXd frf_wrench = sensor_data_->frf_wrench;
  Eigen::VectorXd flf_wrench = sensor_data_->flf_wrench;
  Eigen::VectorXd rrf_wrench = sensor_data_->rrf_wrench;
  Eigen::VectorXd rlf_wrench = sensor_data_->rlf_wrench;

  // Local Z-Force Threshold
  double force_threshold = 10;  // 10 Newtons ~ 1kg. If sensor detects this
                                // force, then we are in contact

  if (fabs(frf_wrench[5]) >= force_threshold) {
    frfoot_contact = true;
  } else {
    frfoot_contact = false;
  }

  if (fabs(flf_wrench[5]) >= force_threshold) {
    flfoot_contact = true;
  } else {
    flfoot_contact = false;
  }
  if (fabs(rrf_wrench[5]) >= force_threshold) {
    rrfoot_contact = true;
  } else {
    rrfoot_contact = false;
  }

  if (fabs(rlf_wrench[5]) >= force_threshold) {
    rlfoot_contact = true;
  } else {
    rlfoot_contact = false;
  }
}

void A1WorldNode::check_foot_contact_by_pos_(bool& frfoot_contact,
                                             bool& flfoot_contact,
                                             bool& rrfoot_contact,
                                             bool& rlfoot_contact) {
  Eigen::VectorXd fr_ = skel_->getBodyNode("FR_foot")->getCOM();
  Eigen::VectorXd fl_ = skel_->getBodyNode("FL_foot")->getCOM();
  Eigen::VectorXd rr_ = skel_->getBodyNode("RR_foot")->getCOM();
  Eigen::VectorXd rl_ = skel_->getBodyNode("RL_foot")->getCOM();
 

  if ((fabs(fl_[2]) < 0.002)) {
    flfoot_contact = true;
    // printf("left contact\n");
  } else {
    flfoot_contact = false;
  }
  if ((fabs(fr_[2]) < 0.002)) {
    frfoot_contact = true;
    // printf("left contact\n");
  } else {
    frfoot_contact = false;
  }
  if ((fabs(rl_[2]) < 0.002)) {
    rlfoot_contact = true;
    // printf("left contact\n");
  } else {
    rlfoot_contact = false;
  }
  if ((fabs(rr_[2]) < 0.002)) {
    rrfoot_contact = true;
    // printf("left contact\n");
  } else {
    rrfoot_contact = false;
  }
}

void A1WorldNode::hold_rot_() {
  Eigen::VectorXd q = skel_->getPositions();
  Eigen::VectorXd v = skel_->getVelocities();
  double kp(200);
  double kd(5);
  trq_cmd_[3] = kp * (-q[3]) + kd * (-v[3]);
  trq_cmd_[4] = kp * (-q[4]) + kd * (-v[4]);
  trq_cmd_[5] = kp * (-q[5]) + kd * (-v[5]);
}

void A1WorldNode::hold_xy_() {
  static double des_x = (skel_->getPositions())[0];
  static double des_y = (skel_->getPositions())[1];
  static double des_xdot(0.);
  static double des_ydot(0.);

  Eigen::VectorXd q = skel_->getPositions();
  Eigen::VectorXd v = skel_->getVelocities();

  double kp(500);
  double kd(100);

  trq_cmd_[0] = kp * (des_x - q[0]) + kd * (des_xdot - v[0]);
  trq_cmd_[1] = kp * (des_y - q[1]) + kd * (des_ydot - v[1]);
}

void A1WorldNode::get_force_torque_data_() {
  Eigen::VectorXd frf_wrench = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd flf_wrench = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd rrf_wrench = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd rlf_wrench = Eigen::VectorXd::Zero(6);

  dart::dynamics::BodyNode* flfoot_bn = skel_->getBodyNode("FL_foot");
  dart::dynamics::BodyNode* frfoot_bn = skel_->getBodyNode("FR_foot");
  dart::dynamics::BodyNode* rlfoot_bn = skel_->getBodyNode("RL_foot");
  dart::dynamics::BodyNode* rrfoot_bn = skel_->getBodyNode("RR_foot");

  const dart::collision::CollisionResult& _result =
      world_->getLastCollisionResult();
/////////////////////////////////////////////////////////////////////////////
  for (const auto& contact : _result.getContacts()) {
    for (const auto& shapeNode :
         frfoot_bn->getShapeNodesWith<dart::dynamics::CollisionAspect>()) {
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
        // T_wc.translation() = contact.point; Since A1 force sensor in foot
        Eigen::Isometry3d T_wa = skel_->getBodyNode("FR_foot")->getTransform(
            dart::dynamics::Frame::World());
        Eigen::Isometry3d T_ca = T_wc.inverse() * T_wa; // T_wc = Identity
        Eigen::MatrixXd AdT_ca = dart::math::getAdTMatrix(T_ca);
        Eigen::VectorXd w_a = Eigen::VectorXd::Zero(6);
        w_a = AdT_ca.transpose() * w_c;
        frf_wrench += w_a;
      }
    }
    for (const auto& shapeNode :
         flfoot_bn->getShapeNodesWith<dart::dynamics::CollisionAspect>()) {
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
        // T_wc.translation() = contact.point; Since A1 force sensor in foot
        Eigen::Isometry3d T_wa = skel_->getBodyNode("FL_foot")->getTransform(
            dart::dynamics::Frame::World());
        Eigen::Isometry3d T_ca = T_wc.inverse() * T_wa; // T_wc = Identity
        Eigen::MatrixXd AdT_ca = dart::math::getAdTMatrix(T_ca);
        Eigen::VectorXd w_a = Eigen::VectorXd::Zero(6);
        w_a = AdT_ca.transpose() * w_c;
        flf_wrench += w_a;
      }
    }
    for (const auto& shapeNode :
         rrfoot_bn->getShapeNodesWith<dart::dynamics::CollisionAspect>()) {
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
        // T_wc.translation() = contact.point; Since A1 force sensor in foot
        Eigen::Isometry3d T_wa = skel_->getBodyNode("RR_foot")->getTransform(
            dart::dynamics::Frame::World());
        Eigen::Isometry3d T_ca = T_wc.inverse() * T_wa; // T_wc = Identity
        Eigen::MatrixXd AdT_ca = dart::math::getAdTMatrix(T_ca);
        Eigen::VectorXd w_a = Eigen::VectorXd::Zero(6);
        w_a = AdT_ca.transpose() * w_c;
        rrf_wrench += w_a;
      }
    }
    for (const auto& shapeNode :
         rlfoot_bn->getShapeNodesWith<dart::dynamics::CollisionAspect>()) {
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
        // T_wc.translation() = contact.point; Since A1 force sensor in foot
        Eigen::Isometry3d T_wa = skel_->getBodyNode("RL_foot")->getTransform(
            dart::dynamics::Frame::World());
        Eigen::Isometry3d T_ca = T_wc.inverse() * T_wa; // T_wc = Identity
        Eigen::MatrixXd AdT_ca = dart::math::getAdTMatrix(T_ca);
        Eigen::VectorXd w_a = Eigen::VectorXd::Zero(6);
        w_a = AdT_ca.transpose() * w_c;
        rlf_wrench += w_a;
      }
    } 
  }

////////////////////////////////////////////////////////////////////////////
  sensor_data_->flf_wrench = flf_wrench;
  sensor_data_->frf_wrench = frf_wrench;
  sensor_data_->rlf_wrench = rlf_wrench;
  sensor_data_->rrf_wrench = rrf_wrench;

}
