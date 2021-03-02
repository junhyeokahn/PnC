#include <Configuration.h>
#include <PnC/A1PnC/A1Interface.hpp>
#include <Simulator/Dart/A1/A1WorldNode.hpp>
#include <Utils/IO/DataManager.hpp>
#include <Utils/IO/IOUtilities.hpp>

A1WorldNode::A1WorldNode(const dart::simulation::WorldPtr& _world)
    : dart::gui::osg::WorldNode(_world),
      count_(0),
      t_(0.0),
      servo_rate_(0.002) {
  world_ = _world;
  skel_ = world_->getSkeleton("a1");
  trq_lb_ = skel_->getForceLowerLimits();
  trq_ub_ = skel_->getForceUpperLimits();
  ground_ = world_->getSkeleton("ground_skeleton");
  dof_ = skel_->getNumDofs();
  trq_cmd_ = Eigen::VectorXd::Zero(dof_);
  pos_cmd_ = Eigen::VectorXd::Zero(dof_);
  vel_cmd_ = Eigen::VectorXd::Zero(dof_);

  interface_ = new A1Interface();
  sensor_data_ = new A1SensorData();
  command_ = new A1Command();

  initial_jpos = Eigen::VectorXd::Zero(12);
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
  sensor_data_->virtual_q = skel_->getPositions().head(6);
  sensor_data_->qdot = skel_->getVelocities().tail(12);
  sensor_data_->virtual_qdot = skel_->getVelocities().head(6);
  sensor_data_->jtrq = skel_->getForces().tail(12);
  // get_force_torque_data_(); // TODO
  get_imu_data_(sensor_data_->imu_ang_vel, sensor_data_->imu_acc);
  check_foot_contact_by_pos_(sensor_data_->frfoot_contact,
                             sensor_data_->flfoot_contact,
                             sensor_data_->rrfoot_contact,
                             sensor_data_->rlfoot_contact);
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
  pos_cmd_.setZero();
  vel_cmd_.setZero();

  // trq_cmd_.tail(12) = command_->jtrq;
  pos_cmd_.tail(12) = command_->q;
  vel_cmd_.tail(12) = command_->qdot;

  Eigen::VectorXd kp(12), kd(12);
  kp << 80., 80., 120., 80., 80., 120., 80., 80., 120, 80., 80., 120.;
  kd << 5., 5., 5., 5., 5., 5., 5., 5., 5., 5., 5., 5.;

  static bool temp = true;
  if(temp){
      initial_jpos = sensor_data_->q;
      temp = false;
  }

  for(int i=0; i<12; ++i){
    // trq_cmd_[i+6] = command_->jtrq[i];
    trq_cmd_[i+6] = kp[i] * (pos_cmd_[i+6] - sensor_data_->q[i]) + kd[i] * (vel_cmd_[i+6] - sensor_data_->qdot[i]) + trq_cmd_[6+i];
    // trq_cmd_[i+6] = kp * (initial_jpos[i] - sensor_data_->q[i]) + kd * (0 - sensor_data_->qdot[i]);
  }
  // myUtils::pretty_print(sensor_data_->virtual_q, std::cout, "Floating Base Pos");
  Eigen::VectorXd temp_vec(12); temp_vec = trq_cmd_.tail(12);
  // myUtils::pretty_print(temp_vec, std::cout, "trq_cmd_ [World Node]");
  // std::cout << "--------------------------------------------------------" << std::endl;
  skel_->setForces(trq_cmd_);


  count_++;

  // reset flags
  resetButtonFlags();
}

void A1WorldNode::get_imu_data_(Eigen::VectorXd& ang_vel,
                                   Eigen::VectorXd& acc) {
  // angvel
  Eigen::VectorXd ang_vel_local =
      skel_->getBodyNode("imu_link")
          ->getSpatialVelocity(dart::dynamics::Frame::World(),
                               skel_->getBodyNode("imu_link")).head(3);
  ang_vel = ang_vel_local;
  Eigen::MatrixXd R_world_imu(3, 3);
  R_world_imu = skel_->getBodyNode("imu_link")->getWorldTransform().linear();
  // acc
  Eigen::Vector3d linear_imu_acc =
      skel_->getBodyNode("imu_link")->getCOMLinearAcceleration();
  Eigen::Vector3d global_grav(0, 0, 9.81);
  // acc = R_world_imu.transpose() * (global_grav + linear_imu_acc);
  acc = R_world_imu.transpose() * (global_grav);

}

void A1WorldNode::check_foot_contact_by_pos_(bool& frfoot_contact,
                                             bool& flfoot_contact,
                                             bool& rrfoot_contact,
                                             bool& rlfoot_contact) {
  Eigen::VectorXd fr_ = skel_->getBodyNode("FR_foot")->getCOM();
  Eigen::VectorXd fl_ = skel_->getBodyNode("FL_foot")->getCOM();
  Eigen::VectorXd rr_ = skel_->getBodyNode("RR_foot")->getCOM();
  Eigen::VectorXd rl_ = skel_->getBodyNode("RL_foot")->getCOM();
  // myUtils::pretty_print(fr_, std::cout, "FR pos");
  // myUtils::pretty_print(fl_, std::cout, "FL pos");
  // myUtils::pretty_print(rr_, std::cout, "RR pos");
  // myUtils::pretty_print(rl_, std::cout, "RL pos");

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


