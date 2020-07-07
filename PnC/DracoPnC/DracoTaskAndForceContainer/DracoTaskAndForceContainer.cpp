#include <PnC/DracoPnC/DracoTaskAndForceContainer/DracoTaskAndForceContainer.hpp>

DracoTaskAndForceContainer::DracoTaskAndForceContainer(RobotSystem* _robot)
    : TaskAndForceContainer(_robot) {
  _InitializeTasks();
  _InitializeContacts();
}

DracoTaskAndForceContainer::~DracoTaskAndForceContainer() {
  _DeleteTasks();
  _DeleteContacts();
}

void DracoTaskAndForceContainer::_InitializeTasks() {
  myUtils::pretty_constructor(2, "Draco Task And Force Container");

  // CoM and Pelvis Tasks
  joint_task_ =
      new BasicTask(robot_, BasicTaskType::JOINT, robot_->getNumActuatedDofs());
  dcm_task_ = new DCMTask(robot_);
  base_ori_task_ =
      new BasicTask(robot_, BasicTaskType::LINKORI, 3, DracoBodyNode::Torso);

  // Set Foot Motion Tasks
  rfoot_center_pos_task_ = new BasicTask(
      robot_, BasicTaskType::ISOLATED_LINKXYZ, 3, DracoBodyNode::rFootCenter);
  lfoot_center_pos_task_ = new BasicTask(
      robot_, BasicTaskType::ISOLATED_LINKXYZ, 3, DracoBodyNode::lFootCenter);
  rfoot_center_ori_task_ =
      new FootLocalRyRzTask(robot_, DracoBodyNode::rFootCenter);
  lfoot_center_ori_task_ =
      new FootLocalRyRzTask(robot_, DracoBodyNode::lFootCenter);

  // Add all tasks initially. Remove later as needed.
  task_list_.push_back(dcm_task_);
  task_list_.push_back(base_ori_task_);

  task_list_.push_back(rfoot_center_pos_task_);
  task_list_.push_back(lfoot_center_pos_task_);
  task_list_.push_back(rfoot_center_ori_task_);
  task_list_.push_back(lfoot_center_ori_task_);
}

void DracoTaskAndForceContainer::_InitializeContacts() {
  rfoot_front_contact_ =
      new PointContactSpec(robot_, DracoBodyNode::rFootFront, 0.7);
  rfoot_back_contact_ =
      new PointContactSpec(robot_, DracoBodyNode::rFootBack, 0.7);
  lfoot_front_contact_ =
      new PointContactSpec(robot_, DracoBodyNode::lFootFront, 0.7);
  lfoot_back_contact_ =
      new PointContactSpec(robot_, DracoBodyNode::lFootBack, 0.7);

  dim_contact_ = rfoot_front_contact_->getDim() +
                 rfoot_back_contact_->getDim() +
                 lfoot_front_contact_->getDim() + lfoot_back_contact_->getDim();
  max_z_ = 500.;

  // Set desired reaction forces
  Fd_des_ = Eigen::VectorXd::Zero(dim_contact_);

  // Add all contacts initially. Remove later as needed.
  contact_list_.push_back(rfoot_front_contact_);
  contact_list_.push_back(rfoot_back_contact_);
  contact_list_.push_back(lfoot_front_contact_);
  contact_list_.push_back(lfoot_back_contact_);
}

void DracoTaskAndForceContainer::_DeleteTasks() {
  delete joint_task_;
  delete dcm_task_;
  delete base_ori_task_;
  delete rfoot_center_pos_task_;
  delete lfoot_center_pos_task_;
  delete rfoot_center_ori_task_;
  delete lfoot_center_ori_task_;
  task_list_.clear();
}

void DracoTaskAndForceContainer::_DeleteContacts() {
  delete rfoot_front_contact_;
  delete rfoot_back_contact_;
  delete lfoot_front_contact_;
  delete lfoot_back_contact_;
  contact_list_.clear();
}

void DracoTaskAndForceContainer::paramInitialization(const YAML::Node& node) {
  try {
    double temp_double;
    Eigen::VectorXd temp_vec;

    // Load Maximum normal force
    myUtils::readParameter(node, "ini_z_force", max_z_);

    // Load Task Gains
    myUtils::readParameter(node, "kp_joint", kp_joint_);
    myUtils::readParameter(node, "kd_joint", kd_joint_);
    myUtils::readParameter(node, "kp_com", kp_com_);
    myUtils::readParameter(node, "kd_com", kd_com_);
    myUtils::readParameter(node, "kp_base_ori", kp_base_ori_);
    myUtils::readParameter(node, "kd_base_ori", kd_base_ori_);
    myUtils::readParameter(node, "kp_foot_pos", kp_foot_pos_);
    myUtils::readParameter(node, "kd_foot_pos", kd_foot_pos_);
    myUtils::readParameter(node, "kp_foot_ori", kp_foot_ori_);
    myUtils::readParameter(node, "kd_foot_ori", kd_foot_ori_);

    // Load Task Hierarchies
    myUtils::readParameter(node, "ini_w_task_com", w_task_com_);
    myUtils::readParameter(node, "ini_w_task_base_ori", w_task_base_ori_);
    myUtils::readParameter(node, "ini_w_task_foot_pos", w_task_foot_pos_);
    myUtils::readParameter(node, "ini_w_task_foot_ori", w_task_foot_ori_);

  } catch (std::runtime_error& e) {
    std::cout << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
    exit(0);
  }

  // Set Task Gains
  joint_task_->setGain(kp_joint_, kd_joint_);
  dcm_task_->setGain(kp_com_, kd_com_);
  base_ori_task_->setGain(kp_base_ori_, kd_base_ori_);
  rfoot_center_pos_task_->setGain(kp_foot_pos_, kd_foot_pos_);
  lfoot_center_pos_task_->setGain(kp_foot_pos_, kd_foot_pos_);
  rfoot_center_ori_task_->setGain(kp_foot_ori_, kd_foot_ori_);
  lfoot_center_ori_task_->setGain(kp_foot_ori_, kd_foot_ori_);

  // Set Task Hierarchies
  dcm_task_->setHierarchyWeight(w_task_com_);
  base_ori_task_->setHierarchyWeight(w_task_base_ori_);
  rfoot_center_pos_task_->setHierarchyWeight(w_task_foot_pos_);
  rfoot_center_ori_task_->setHierarchyWeight(w_task_foot_ori_);
  lfoot_center_pos_task_->setHierarchyWeight(w_task_foot_pos_);
  lfoot_center_ori_task_->setHierarchyWeight(w_task_foot_ori_);

  // Set Maximum Forces
  ((PointContactSpec*)rfoot_front_contact_)->setMaxFz(max_z_);
  ((PointContactSpec*)rfoot_back_contact_)->setMaxFz(max_z_);
  ((PointContactSpec*)lfoot_front_contact_)->setMaxFz(max_z_);
  ((PointContactSpec*)lfoot_back_contact_)->setMaxFz(max_z_);
}
