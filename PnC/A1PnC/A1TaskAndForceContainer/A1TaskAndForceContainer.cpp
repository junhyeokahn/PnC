#include <PnC/A1PnC/A1TaskAndForceContainer/A1TaskAndForceContainer.hpp>

A1TaskAndForceContainer::A1TaskAndForceContainer(RobotSystem* _robot)
    : TaskAndForceContainer(_robot) {
  _InitializeTasks();
  _InitializeContacts();
}

A1TaskAndForceContainer::~A1TaskAndForceContainer() {
  _DeleteTasks();
  _DeleteContacts();
}

void A1TaskAndForceContainer::_InitializeTasks() {
  myUtils::pretty_constructor(2, "A1 Task And Force Container");

  // CoM and Pelvis Tasks
  // joint_task_ =
  //     new BasicTask(robot_, BasicTaskType::JOINT,
  //     robot_->getNumActuatedDofs());
  com_task_ = new CoMxyz(robot_);
  base_ori_task_ =
      new BasicTask(robot_, BasicTaskType::LINKORI, 3, A1BodyNode::trunk);

  // Set Foot Motion Tasks
  frfoot_pos_task_ = new BasicTask(robot_, BasicTaskType::ISOLATED_LINKXYZ, 3,
                                   A1BodyNode::FR_foot);
  flfoot_pos_task_ = new BasicTask(robot_, BasicTaskType::ISOLATED_LINKXYZ, 3,
                                   A1BodyNode::FL_foot);
  rrfoot_pos_task_ = new BasicTask(robot_, BasicTaskType::ISOLATED_LINKXYZ, 3,
                                   A1BodyNode::RR_foot);
  rlfoot_pos_task_ = new BasicTask(robot_, BasicTaskType::ISOLATED_LINKXYZ, 3,
                                   A1BodyNode::RL_foot);

  // Add all tasks initially. Remove later as needed.
  task_list_.push_back(com_task_);
  task_list_.push_back(base_ori_task_);

  task_list_.push_back(frfoot_pos_task_);
  task_list_.push_back(flfoot_pos_task_);
  task_list_.push_back(rrfoot_pos_task_);
  task_list_.push_back(rlfoot_pos_task_);
}

void A1TaskAndForceContainer::_InitializeContacts() {
  frfoot_contact_ = new PointContactSpec(robot_, A1BodyNode::FR_foot, 0.3);
  flfoot_contact_ = new PointContactSpec(robot_, A1BodyNode::FL_foot, 0.3);
  rrfoot_contact_ = new PointContactSpec(robot_, A1BodyNode::RR_foot, 0.3);
  rlfoot_contact_ = new PointContactSpec(robot_, A1BodyNode::RL_foot, 0.3);

  dim_contact_ = frfoot_contact_->getDim() + flfoot_contact_->getDim() +
                 rrfoot_contact_->getDim() + rlfoot_contact_->getDim();

  max_z_ = 123.;

  // Set desired reaction force vector for each foot
  Fr_des_ = Eigen::Vector3d(0., 0., max_z_/4.);
  frfoot_contact_->setRFDesired(Fr_des_);
  flfoot_contact_->setRFDesired(Fr_des_);
  rrfoot_contact_->setRFDesired(Fr_des_);
  rlfoot_contact_->setRFDesired(Fr_des_);

  // Add all contacts initially. Remove later as needed.
  contact_list_.push_back(frfoot_contact_);
  contact_list_.push_back(flfoot_contact_);
  contact_list_.push_back(rrfoot_contact_);
  contact_list_.push_back(rlfoot_contact_);
}

void A1TaskAndForceContainer::_DeleteTasks() {
  // delete joint_task_;
  delete com_task_;
  delete base_ori_task_;
  delete frfoot_pos_task_;
  delete flfoot_pos_task_;
  delete rrfoot_pos_task_;
  delete rlfoot_pos_task_;
  task_list_.clear();
}

void A1TaskAndForceContainer::_DeleteContacts() {
  delete frfoot_contact_;
  delete flfoot_contact_;
  delete rrfoot_contact_;
  delete rlfoot_contact_;
  contact_list_.clear();
}

void A1TaskAndForceContainer::paramInitialization(const YAML::Node& node) {
  try {
    double temp_double;
    Eigen::VectorXd temp_vec;

    // Load Maximum normal force
    myUtils::readParameter(node, "ini_z_force", max_z_);

    // Load Task Gains
    // myUtils::readParameter(node, "kp_joint", kp_joint_);
    // myUtils::readParameter(node, "kd_joint", kd_joint_);
    myUtils::readParameter(node, "kp_com", kp_com_);
    myUtils::readParameter(node, "kd_com", kd_com_);
    myUtils::readParameter(node, "kp_base_ori", kp_base_ori_);
    myUtils::readParameter(node, "kd_base_ori", kd_base_ori_);
    myUtils::readParameter(node, "kp_foot_pos", kp_foot_pos_);
    myUtils::readParameter(node, "kd_foot_pos", kd_foot_pos_);

    // Load Task Hierarchies
    myUtils::readParameter(node, "w_task_com", w_task_com_);
    myUtils::readParameter(node, "w_task_base_ori", w_task_base_ori_);
    myUtils::readParameter(node, "w_task_foot_pos", w_task_foot_pos_);

  } catch (std::runtime_error& e) {
    std::cout << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
    exit(0);
  }

  // Set Task Gains
  // joint_task_->setGain(kp_joint_, kd_joint_);
  com_task_->setGain(kp_com_, kd_com_);
  base_ori_task_->setGain(kp_base_ori_, kd_base_ori_);
  frfoot_pos_task_->setGain(kp_foot_pos_, kd_foot_pos_);
  flfoot_pos_task_->setGain(kp_foot_pos_, kd_foot_pos_);
  rrfoot_pos_task_->setGain(kp_foot_pos_, kd_foot_pos_);
  rlfoot_pos_task_->setGain(kp_foot_pos_, kd_foot_pos_);

  // Set Task Hierarchies
  com_task_->setHierarchyWeight(w_task_com_);
  base_ori_task_->setHierarchyWeight(w_task_base_ori_);
  frfoot_pos_task_->setHierarchyWeight(w_task_foot_pos_);
  flfoot_pos_task_->setHierarchyWeight(w_task_foot_pos_);
  rrfoot_pos_task_->setHierarchyWeight(w_task_foot_pos_);
  rlfoot_pos_task_->setHierarchyWeight(w_task_foot_pos_);

  // Set Maximum Forces
  ((PointContactSpec*)frfoot_contact_)->setMaxFz(max_z_);
  ((PointContactSpec*)rrfoot_contact_)->setMaxFz(max_z_);
  ((PointContactSpec*)flfoot_contact_)->setMaxFz(max_z_);
  ((PointContactSpec*)rlfoot_contact_)->setMaxFz(max_z_);
}
