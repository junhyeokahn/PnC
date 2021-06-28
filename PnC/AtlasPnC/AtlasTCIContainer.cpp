#include <PnC/AtlasPnC/AtlasTCIContainer.hpp>

AtlasTCIContainer::AtlasTCIContainer(RobotSystem *_robot)
    : TCIContainer(_robot) {

  YAML::Node cfg = YAML::LoadFile(THIS_COM "Config/Atlas/pnc.yaml");

  // Initialize Task
  com_task = new BasicTask(robot_, BasicTask::Type::COM, 3);
  com_task->kp = myUtils::readParameter<Eigen::VectorXd>(cfg["wbc"], "kp_com");
  com_task->kd = myUtils::readParameter<Eigen::VectorXd>(cfg["wbc"], "kd_com");
  com_task->w_hierarchy = myUtils::readParameter<double>(cfg["wbc"], "w_com");

  pelvis_ori_task =
      new BasicTask(robot_, BasicTask::Type::LINK_ORI, 3, {"pelvis_com"});
  pelvis_ori_task->kp =
      myUtils::readParameter<Eigen::VectorXd>(cfg["wbc"], "kp_pelvis_ori");
  pelvis_ori_task->kd =
      myUtils::readParameter<Eigen::VectorXd>(cfg["wbc"], "kd_pelvis_ori");
  pelvis_ori_task->w_hierarchy =
      myUtils::readParameter<double>(cfg["wbc"], "w_pelvis_ori");

  std::vector<std::string> upper_body_joint = {
      "back_bkx",  "back_bky",  "back_bkz",  "l_arm_elx", "l_arm_ely",
      "l_arm_shx", "l_arm_shz", "l_arm_wrx", "l_arm_wry", "l_arm_wry2",
      "neck_ry",   "r_arm_elx", "r_arm_ely", "r_arm_shx", "r_arm_shz",
      "r_arm_wrx", "r_arm_wry", "r_arm_wry2"};
  upper_body_task = new BasicTask(robot_, BasicTask::Type::SELECTED_JOINT,
                                  upper_body_joint.size(), upper_body_joint);
  upper_body_task->kp = myUtils::readParameter<Eigen::VectorXd>(
      cfg["wbc"], "kp_upper_body_joint");
  upper_body_task->kd = myUtils::readParameter<Eigen::VectorXd>(
      cfg["wbc"], "kd_upper_body_joint");
  upper_body_task->w_hierarchy =
      myUtils::readParameter<double>(cfg["wbc"], "w_upper_body_joint");

  rfoot_pos_task =
      new BasicTask(robot_, BasicTask::Type::LINK_XYZ, 3, {"r_sole"});
  rfoot_pos_task->kp =
      myUtils::readParameter<Eigen::VectorXd>(cfg["wbc"], "kp_foot_pos");
  rfoot_pos_task->kd =
      myUtils::readParameter<Eigen::VectorXd>(cfg["wbc"], "kd_foot_pos");
  rfoot_pos_task->w_hierarchy =
      myUtils::readParameter<double>(cfg["wbc"], "w_contact_foot");

  rfoot_ori_task =
      new BasicTask(robot_, BasicTask::Type::LINK_ORI, 3, {"r_sole"});
  rfoot_ori_task->kp =
      myUtils::readParameter<Eigen::VectorXd>(cfg["wbc"], "kp_foot_ori");
  rfoot_ori_task->kd =
      myUtils::readParameter<Eigen::VectorXd>(cfg["wbc"], "kd_foot_ori");
  rfoot_ori_task->w_hierarchy =
      myUtils::readParameter<double>(cfg["wbc"], "w_contact_foot");

  lfoot_pos_task =
      new BasicTask(robot_, BasicTask::Type::LINK_XYZ, 3, {"l_sole"});
  lfoot_pos_task->kp =
      myUtils::readParameter<Eigen::VectorXd>(cfg["wbc"], "kp_foot_pos");
  lfoot_pos_task->kd =
      myUtils::readParameter<Eigen::VectorXd>(cfg["wbc"], "kd_foot_pos");
  lfoot_pos_task->w_hierarchy =
      myUtils::readParameter<double>(cfg["wbc"], "w_contact_foot");

  lfoot_ori_task =
      new BasicTask(robot_, BasicTask::Type::LINK_ORI, 3, {"l_sole"});
  lfoot_ori_task->kp =
      myUtils::readParameter<Eigen::VectorXd>(cfg["wbc"], "kp_foot_ori");
  lfoot_ori_task->kd =
      myUtils::readParameter<Eigen::VectorXd>(cfg["wbc"], "kd_foot_ori");
  lfoot_ori_task->w_hierarchy =
      myUtils::readParameter<double>(cfg["wbc"], "w_contact_foot");

  task_list.push_back(com_task);
  task_list.push_back(pelvis_ori_task);
  task_list.push_back(upper_body_task);
  task_list.push_back(rfoot_pos_task);
  task_list.push_back(rfoot_ori_task);
  task_list.push_back(lfoot_pos_task);
  task_list.push_back(lfoot_ori_task);

  // Initialize Contact
  rfoot_contact = new SurfaceContact(robot_, "r_sole", 0.11, 0.065, 0.3);
  rfoot_contact->rf_z_max = 1e-3;
  lfoot_contact = new SurfaceContact(robot_, "l_sole", 0.11, 0.065, 0.3);
  lfoot_contact->rf_z_max = 1e-3;

  contact_list.push_back(rfoot_contact);
  contact_list.push_back(lfoot_contact);

  // Initialize Internal Constraint
}

AtlasTCIContainer::~AtlasTCIContainer() {
  delete com_task;
  delete pelvis_ori_task;
  delete upper_body_task;
  delete rfoot_pos_task;
  delete rfoot_ori_task;
  delete lfoot_pos_task;
  delete lfoot_ori_task;

  delete rfoot_contact;
  delete lfoot_contact;
}
