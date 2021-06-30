#include <PnC/draco_pnc/draco_tci_container.hpp>

DracoTCIContainer::DracoTCIContainer(RobotSystem *_robot)
    : TCIContainer(_robot) {

  YAML::Node cfg = YAML::LoadFile(THIS_COM "Config/draco/pnc.yaml");

  // Initialize Task
  joint_task = new JointTask(robot_);

  com_task = new CenterOfMassTask(robot_);
  com_task->kp = myUtils::readParameter<Eigen::VectorXd>(cfg["wbc"], "kp_com");
  com_task->kd = myUtils::readParameter<Eigen::VectorXd>(cfg["wbc"], "kd_com");
  com_task->w_hierarchy = myUtils::readParameter<double>(cfg["wbc"], "w_com");

  torso_ori_task = new LinkOriTask(robot_, {"torso_com_link"});
  torso_ori_task->kp =
      myUtils::readParameter<Eigen::VectorXd>(cfg["wbc"], "kp_torso_ori");
  torso_ori_task->kd =
      myUtils::readParameter<Eigen::VectorXd>(cfg["wbc"], "kd_torso_ori");
  torso_ori_task->w_hierarchy =
      myUtils::readParameter<double>(cfg["wbc"], "w_torso_ori");

  std::vector<std::string> upper_body_joint = {
      "neck_pitch",    "l_shoulder_fe", "l_shoulder_aa", "l_shoulder_ie",
      "l_elbow_fe",    "l_wrist_ps",    "l_wrist_pitch", "r_shoulder_fe",
      "r_shoulder_aa", "r_shoulder_ie", "r_elbow_fe",    "r_wrist_ps",
      "r_wrist_pitch"};
  upper_body_task = new SelectedJointTask(robot_, upper_body_joint);
  upper_body_task->kp = myUtils::readParameter<Eigen::VectorXd>(
      cfg["wbc"], "kp_upper_body_joint");
  upper_body_task->kd = myUtils::readParameter<Eigen::VectorXd>(
      cfg["wbc"], "kd_upper_body_joint");
  upper_body_task->w_hierarchy =
      myUtils::readParameter<double>(cfg["wbc"], "w_upper_body_joint");

  rfoot_pos_task = new LinkPosTask(robot_, {"r_foot_contact"});
  rfoot_pos_task->kp =
      myUtils::readParameter<Eigen::VectorXd>(cfg["wbc"], "kp_foot_pos");
  rfoot_pos_task->kd =
      myUtils::readParameter<Eigen::VectorXd>(cfg["wbc"], "kd_foot_pos");
  rfoot_pos_task->w_hierarchy =
      myUtils::readParameter<double>(cfg["wbc"], "w_contact_foot");

  rfoot_ori_task = new LinkOriTask(robot_, {"r_foot_contact"});
  rfoot_ori_task->kp =
      myUtils::readParameter<Eigen::VectorXd>(cfg["wbc"], "kp_foot_ori");
  rfoot_ori_task->kd =
      myUtils::readParameter<Eigen::VectorXd>(cfg["wbc"], "kd_foot_ori");
  rfoot_ori_task->w_hierarchy =
      myUtils::readParameter<double>(cfg["wbc"], "w_contact_foot");

  lfoot_pos_task = new LinkPosTask(robot_, {"l_foot_contact"});
  lfoot_pos_task->kp =
      myUtils::readParameter<Eigen::VectorXd>(cfg["wbc"], "kp_foot_pos");
  lfoot_pos_task->kd =
      myUtils::readParameter<Eigen::VectorXd>(cfg["wbc"], "kd_foot_pos");
  lfoot_pos_task->w_hierarchy =
      myUtils::readParameter<double>(cfg["wbc"], "w_contact_foot");

  lfoot_ori_task = new LinkOriTask(robot_, {"l_foot_contact"});
  lfoot_ori_task->kp =
      myUtils::readParameter<Eigen::VectorXd>(cfg["wbc"], "kp_foot_ori");
  lfoot_ori_task->kd =
      myUtils::readParameter<Eigen::VectorXd>(cfg["wbc"], "kd_foot_ori");
  lfoot_ori_task->w_hierarchy =
      myUtils::readParameter<double>(cfg["wbc"], "w_contact_foot");

  task_list.push_back(com_task);
  task_list.push_back(torso_ori_task);
  task_list.push_back(upper_body_task);
  task_list.push_back(rfoot_pos_task);
  task_list.push_back(rfoot_ori_task);
  task_list.push_back(lfoot_pos_task);
  task_list.push_back(lfoot_ori_task);

  // Initialize Contact
  rfoot_contact = new SurfaceContact(robot_, "r_foot_contact", 0.08, 0.02, 0.5);
  rfoot_contact->rf_z_max = 1e-3;
  lfoot_contact = new SurfaceContact(robot_, "l_foot_contact", 0.08, 0.02, 0.5);
  lfoot_contact->rf_z_max = 1e-3;

  contact_list.push_back(rfoot_contact);
  contact_list.push_back(lfoot_contact);

  // Initialize Internal Constraint
}

DracoTCIContainer::~DracoTCIContainer() {
  delete joint_task;
  delete com_task;
  delete torso_ori_task;
  delete upper_body_task;
  delete rfoot_pos_task;
  delete rfoot_ori_task;
  delete lfoot_pos_task;
  delete lfoot_ori_task;

  delete rfoot_contact;
  delete lfoot_contact;
}
