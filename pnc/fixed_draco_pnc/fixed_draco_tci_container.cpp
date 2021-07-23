#include <pnc/fixed_draco_pnc/fixed_draco_tci_container.hpp>

FixedDracoTCIContainer::FixedDracoTCIContainer(RobotSystem *_robot)
    : TCIContainer(_robot) {
  util::PrettyConstructor(2, "FixedDracoTCIContainer");

  YAML::Node cfg = YAML::LoadFile(THIS_COM "config/fixed_draco/pnc.yaml");

  // Initialize Task
  joint_task = new JointTask(robot_);

  std::vector<std::string> upper_body_joint = {
      "neck_pitch",    "l_shoulder_fe", "l_shoulder_aa", "l_shoulder_ie",
      "l_elbow_fe",    "l_wrist_ps",    "l_wrist_pitch", "r_shoulder_fe",
      "r_shoulder_aa", "r_shoulder_ie", "r_elbow_fe",    "r_wrist_ps",
      "r_wrist_pitch"};
  upper_body_task = new SelectedJointTask(robot_, upper_body_joint);
  upper_body_task->kp =
      util::ReadParameter<Eigen::VectorXd>(cfg["wbc"], "kp_upper_body_joint");
  upper_body_task->kd =
      util::ReadParameter<Eigen::VectorXd>(cfg["wbc"], "kd_upper_body_joint");
  upper_body_task->w_hierarchy =
      util::ReadParameter<double>(cfg["wbc"], "w_upper_body_joint");

  rfoot_pos_task = new LinkPosTask(robot_, {"r_foot_contact"});
  rfoot_pos_task->kp =
      util::ReadParameter<Eigen::VectorXd>(cfg["wbc"], "kp_foot_pos");
  rfoot_pos_task->kd =
      util::ReadParameter<Eigen::VectorXd>(cfg["wbc"], "kd_foot_pos");
  rfoot_pos_task->w_hierarchy =
      util::ReadParameter<double>(cfg["wbc"], "w_foot");

  rfoot_ori_task = new LinkOriTask(robot_, {"r_foot_contact"});
  rfoot_ori_task->kp =
      util::ReadParameter<Eigen::VectorXd>(cfg["wbc"], "kp_foot_ori");
  rfoot_ori_task->kd =
      util::ReadParameter<Eigen::VectorXd>(cfg["wbc"], "kd_foot_ori");
  rfoot_ori_task->w_hierarchy =
      util::ReadParameter<double>(cfg["wbc"], "w_foot");

  lfoot_pos_task = new LinkPosTask(robot_, {"l_foot_contact"});
  lfoot_pos_task->kp =
      util::ReadParameter<Eigen::VectorXd>(cfg["wbc"], "kp_foot_pos");
  lfoot_pos_task->kd =
      util::ReadParameter<Eigen::VectorXd>(cfg["wbc"], "kd_foot_pos");
  lfoot_pos_task->w_hierarchy =
      util::ReadParameter<double>(cfg["wbc"], "w_foot");

  lfoot_ori_task = new LinkOriTask(robot_, {"l_foot_contact"});
  lfoot_ori_task->kp =
      util::ReadParameter<Eigen::VectorXd>(cfg["wbc"], "kp_foot_ori");
  lfoot_ori_task->kd =
      util::ReadParameter<Eigen::VectorXd>(cfg["wbc"], "kd_foot_ori");
  lfoot_ori_task->w_hierarchy =
      util::ReadParameter<double>(cfg["wbc"], "w_foot");

  task_list.push_back(upper_body_task);
  task_list.push_back(rfoot_pos_task);
  task_list.push_back(rfoot_ori_task);
  task_list.push_back(lfoot_pos_task);
  task_list.push_back(lfoot_ori_task);

  // Initialize Contact

  // Initialize Internal Constraint
  rolling_joint_constraint = new FixedDracoRollingJointConstraint(robot_);
  internal_constraint_list.push_back(rolling_joint_constraint);
}

FixedDracoTCIContainer::~FixedDracoTCIContainer() {
  delete joint_task;
  delete upper_body_task;
  delete rfoot_pos_task;
  delete rfoot_ori_task;
  delete lfoot_pos_task;
  delete lfoot_ori_task;

  delete rolling_joint_constraint;
}
