#include <pnc/fixed_draco_pnc/fixed_draco_tci_container.hpp>

FixedDracoTCIContainer::FixedDracoTCIContainer(RobotSystem *_robot)
    : TCIContainer(_robot) {
  util::PrettyConstructor(2, "FixedDracoTCIContainer");

  YAML::Node cfg = YAML::LoadFile(THIS_COM "config/fixed_draco/pnc.yaml");

  // Initialize Task
  joint_task = new JointTask(robot_);

  // std::vector<std::string> upper_body_joint = {
  //"neck_pitch",    "l_shoulder_fe", "l_shoulder_aa", "l_shoulder_ie",
  //"l_elbow_fe",    "l_wrist_ps",    "l_wrist_pitch", "r_shoulder_fe",
  //"r_shoulder_aa", "r_shoulder_ie", "r_elbow_fe",    "r_wrist_ps",
  //"r_wrist_pitch"};
  // upper_body_task = new SelectedJointTask(robot_, upper_body_joint);
  // upper_body_task->kp = Eigen::VectorXd::Zero(upper_body_joint.size());
  // upper_body_task->kd = Eigen::VectorXd::Zero(upper_body_joint.size());
  // for (int i = 0; i < upper_body_joint.size(); ++i) {
  // upper_body_task->kp[i] = util::ReadParameter<double>(
  // cfg["wbc"]["task"]["upper_body_joint"][upper_body_joint[i]], "kp");
  // upper_body_task->kd[i] = util::ReadParameter<double>(
  // cfg["wbc"]["task"]["upper_body_joint"][upper_body_joint[i]], "kd");
  //}
  // upper_body_task->w_hierarchy = util::ReadParameter<double>(
  // cfg["wbc"]["task"]["upper_body_joint"], "weight");
  std::vector<std::string> neck_joint = {"neck_pitch"};
  neck_task = new SelectedJointTask(robot_, neck_joint);
  neck_task->kp = Eigen::VectorXd::Zero(neck_joint.size());
  neck_task->kd = Eigen::VectorXd::Zero(neck_joint.size());
  for (int i = 0; i < neck_joint.size(); ++i) {
    neck_task->kp[i] =
        util::ReadParameter<double>(cfg["wbc"]["task"]["neck"], "kp");
  }
  for (int i = 0; i < neck_joint.size(); ++i) {
    neck_task->kd[i] =
        util::ReadParameter<double>(cfg["wbc"]["task"]["neck"], "kd");
  }
  neck_task->w_hierarchy = util::ReadParameter<Eigen::VectorXd>(
      cfg["wbc"]["task"]["neck"], "weight");

  rfoot_pos_task = new LinkPosTask(robot_, {"r_foot_contact"});
  rfoot_pos_task->kp = util::ReadParameter<Eigen::VectorXd>(
      cfg["wbc"]["task"]["foot_pos"], "kp");
  rfoot_pos_task->kd = util::ReadParameter<Eigen::VectorXd>(
      cfg["wbc"]["task"]["foot_pos"], "kd");
  rfoot_pos_task->w_hierarchy = util::ReadParameter<Eigen::VectorXd>(
      cfg["wbc"]["task"]["foot_pos"], "weight");

  rfoot_ori_task = new LinkOriTask(robot_, {"r_foot_contact"});
  rfoot_ori_task->kp = util::ReadParameter<Eigen::VectorXd>(
      cfg["wbc"]["task"]["foot_ori"], "kp");
  rfoot_ori_task->kd = util::ReadParameter<Eigen::VectorXd>(
      cfg["wbc"]["task"]["foot_ori"], "kd");
  rfoot_ori_task->w_hierarchy = util::ReadParameter<Eigen::VectorXd>(
      cfg["wbc"]["task"]["foot_ori"], "weight");

  lfoot_pos_task = new LinkPosTask(robot_, {"l_foot_contact"});
  lfoot_pos_task->kp = util::ReadParameter<Eigen::VectorXd>(
      cfg["wbc"]["task"]["foot_pos"], "kp");
  lfoot_pos_task->kd = util::ReadParameter<Eigen::VectorXd>(
      cfg["wbc"]["task"]["foot_pos"], "kd");
  lfoot_pos_task->w_hierarchy = util::ReadParameter<Eigen::VectorXd>(
      cfg["wbc"]["task"]["foot_pos"], "weight");

  lfoot_ori_task = new LinkOriTask(robot_, {"l_foot_contact"});
  lfoot_ori_task->kp = util::ReadParameter<Eigen::VectorXd>(
      cfg["wbc"]["task"]["foot_ori"], "kp");
  lfoot_ori_task->kd = util::ReadParameter<Eigen::VectorXd>(
      cfg["wbc"]["task"]["foot_ori"], "kd");
  lfoot_ori_task->w_hierarchy = util::ReadParameter<Eigen::VectorXd>(
      cfg["wbc"]["task"]["foot_ori"], "weight");

  rhand_pos_task = new LinkPosTask(robot_, {"r_hand_contact"});
  rhand_pos_task->kp = util::ReadParameter<Eigen::VectorXd>(
      cfg["wbc"]["task"]["hand_pos"], "kp");
  rhand_pos_task->kd = util::ReadParameter<Eigen::VectorXd>(
      cfg["wbc"]["task"]["hand_pos"], "kd");
  rhand_pos_task->w_hierarchy = util::ReadParameter<Eigen::VectorXd>(
      cfg["wbc"]["task"]["hand_pos"], "weight");

  rhand_ori_task = new LinkOriTask(robot_, {"r_hand_contact"});
  rhand_ori_task->kp = util::ReadParameter<Eigen::VectorXd>(
      cfg["wbc"]["task"]["hand_ori"], "kp");
  rhand_ori_task->kd = util::ReadParameter<Eigen::VectorXd>(
      cfg["wbc"]["task"]["hand_ori"], "kd");
  rhand_ori_task->w_hierarchy = util::ReadParameter<Eigen::VectorXd>(
      cfg["wbc"]["task"]["hand_ori"], "weight");

  lhand_pos_task = new LinkPosTask(robot_, {"l_hand_contact"});
  lhand_pos_task->kp = util::ReadParameter<Eigen::VectorXd>(
      cfg["wbc"]["task"]["hand_pos"], "kp");
  lhand_pos_task->kd = util::ReadParameter<Eigen::VectorXd>(
      cfg["wbc"]["task"]["hand_pos"], "kd");
  lhand_pos_task->w_hierarchy = util::ReadParameter<Eigen::VectorXd>(
      cfg["wbc"]["task"]["hand_pos"], "weight");

  lhand_ori_task = new LinkOriTask(robot_, {"l_hand_contact"});
  lhand_ori_task->kp = util::ReadParameter<Eigen::VectorXd>(
      cfg["wbc"]["task"]["hand_ori"], "kp");
  lhand_ori_task->kd = util::ReadParameter<Eigen::VectorXd>(
      cfg["wbc"]["task"]["hand_ori"], "kd");
  lhand_ori_task->w_hierarchy = util::ReadParameter<Eigen::VectorXd>(
      cfg["wbc"]["task"]["hand_ori"], "weight");

  // task_list.push_back(upper_body_task);
  task_list.push_back(neck_task);
  task_list.push_back(rfoot_pos_task);
  task_list.push_back(rfoot_ori_task);
  task_list.push_back(lfoot_pos_task);
  task_list.push_back(lfoot_ori_task);
  task_list.push_back(rhand_pos_task);
  task_list.push_back(rhand_ori_task);
  task_list.push_back(lhand_pos_task);
  task_list.push_back(lhand_ori_task);
  // Initialize Contact

  // Initialize Internal Constraint
  rolling_joint_constraint = new FixedDracoRollingJointConstraint(robot_);
  internal_constraint_list.push_back(rolling_joint_constraint);
}

FixedDracoTCIContainer::~FixedDracoTCIContainer() {
  delete joint_task;
  // delete upper_body_task;
  delete neck_task;
  delete rfoot_pos_task;
  delete rfoot_ori_task;
  delete lfoot_pos_task;
  delete lfoot_ori_task;
  delete rhand_pos_task;
  delete rhand_ori_task;
  delete lhand_pos_task;
  delete lhand_ori_task;

  delete rolling_joint_constraint;
}
