#include "pnc/draco_pnc/draco_tci_container.hpp"

#include "pnc/draco_pnc/draco_task/draco_angular_momentum_task.hpp"
#include "pnc/draco_pnc/draco_task/draco_com_task.hpp"

DracoTCIContainer::DracoTCIContainer(RobotSystem *_robot)
    : TCIContainer(_robot) {
  util::PrettyConstructor(2, "DracoTCIContainer");

  YAML::Node cfg = YAML::LoadFile(THIS_COM "config/draco/pnc.yaml");

  bool b_exp = util::ReadParameter<bool>(cfg, "b_exp");
  std::string gain_prefix;
  if (b_exp) {
    gain_prefix = "exp_";
  } else {
    gain_prefix = "sim_";
  }

  // Initialize Task
  joint_task = new JointTask(robot_);

  int feedback_source = util::ReadParameter<int>(cfg["wbc"]["task"],
                                                 "com_control_feedback_source");
  int feedback_height_target = util::ReadParameter<int>(
      cfg["wbc"]["task"], "com_control_feedback_height_target");
  com_task = new DracoCenterOfMassTask(robot_, feedback_source,
                                       feedback_height_target);
  if (feedback_source == feedback_source::kCom) {
    com_task->kp = util::ReadParameter<Eigen::VectorXd>(
        cfg["wbc"]["task"]["com"], gain_prefix + "kp");
    com_task->kd = util::ReadParameter<Eigen::VectorXd>(
        cfg["wbc"]["task"]["com"], gain_prefix + "kd");
    com_task->w_hierarchy =
        util::ReadParameter<double>(cfg["wbc"]["task"]["com"], "weight");
  } else if (feedback_source == feedback_source::kIcp) {
    com_task->kp = util::ReadParameter<Eigen::VectorXd>(
        cfg["wbc"]["task"]["icp"], gain_prefix + "kp");
    com_task->kd = util::ReadParameter<Eigen::VectorXd>(
        cfg["wbc"]["task"]["icp"], gain_prefix + "kd");
    com_task->ki = util::ReadParameter<Eigen::VectorXd>(
        cfg["wbc"]["task"]["icp"], gain_prefix + "ki");
    com_task->w_hierarchy =
        util::ReadParameter<double>(cfg["wbc"]["task"]["icp"], "weight");
  } else {
    assert(false);
  }

  cam_task = new DracoAngularMomentumTask(robot_);
  cam_task->kp = util::ReadParameter<Eigen::VectorXd>(cfg["wbc"]["task"]["cam"],
                                                      gain_prefix + "kp");
  cam_task->kd = util::ReadParameter<Eigen::VectorXd>(cfg["wbc"]["task"]["cam"],
                                                      gain_prefix + "kd");
  cam_task->w_hierarchy =
      util::ReadParameter<double>(cfg["wbc"]["task"]["cam"], "weight");

  torso_ori_task = new LinkOriTask(robot_, {"torso_com_link"});
  torso_ori_task->kp = util::ReadParameter<Eigen::VectorXd>(
      cfg["wbc"]["task"]["torso_ori"], gain_prefix + "kp");
  torso_ori_task->kd = util::ReadParameter<Eigen::VectorXd>(
      cfg["wbc"]["task"]["torso_ori"], gain_prefix + "kd");
  torso_ori_task->w_hierarchy =
      util::ReadParameter<double>(cfg["wbc"]["task"]["torso_ori"], "weight");

  std::vector<std::string> upper_body_joint = {
      "neck_pitch",    "l_shoulder_fe", "l_shoulder_aa", "l_shoulder_ie",
      "l_elbow_fe",    "l_wrist_ps",    "l_wrist_pitch", "r_shoulder_fe",
      "r_shoulder_aa", "r_shoulder_ie", "r_elbow_fe",    "r_wrist_ps",
      "r_wrist_pitch"};
  upper_body_task = new SelectedJointTask(robot_, upper_body_joint);
  upper_body_task->kp = Eigen::VectorXd::Zero(upper_body_joint.size());
  upper_body_task->kd = Eigen::VectorXd::Zero(upper_body_joint.size());
  for (int i = 0; i < upper_body_joint.size(); ++i) {
    upper_body_task->kp[i] = util::ReadParameter<double>(
        cfg["wbc"]["task"]["upper_body_joint"][upper_body_joint[i]],
        gain_prefix + "kp");
    upper_body_task->kd[i] = util::ReadParameter<double>(
        cfg["wbc"]["task"]["upper_body_joint"][upper_body_joint[i]],
        gain_prefix + "kd");
  }
  upper_body_task->w_hierarchy = util::ReadParameter<double>(
      cfg["wbc"]["task"]["upper_body_joint"], "weight");

  rfoot_pos_task = new LinkPosTask(robot_, {"r_foot_contact"});
  rfoot_pos_task->kp = util::ReadParameter<Eigen::VectorXd>(
      cfg["wbc"]["task"]["foot_pos"], gain_prefix + "kp");
  rfoot_pos_task->kd = util::ReadParameter<Eigen::VectorXd>(
      cfg["wbc"]["task"]["foot_pos"], gain_prefix + "kd");
  rfoot_pos_task->w_hierarchy = util::ReadParameter<double>(
      cfg["wbc"]["task"]["foot_pos"], "weight_at_swing");

  rfoot_ori_task = new LinkOriTask(robot_, {"r_foot_contact"});
  rfoot_ori_task->kp = util::ReadParameter<Eigen::VectorXd>(
      cfg["wbc"]["task"]["foot_ori"], gain_prefix + "kp");
  rfoot_ori_task->kd = util::ReadParameter<Eigen::VectorXd>(
      cfg["wbc"]["task"]["foot_ori"], gain_prefix + "kd");
  rfoot_ori_task->w_hierarchy = util::ReadParameter<double>(
      cfg["wbc"]["task"]["foot_ori"], "weight_at_swing");

  lfoot_pos_task = new LinkPosTask(robot_, {"l_foot_contact"});
  lfoot_pos_task->kp = util::ReadParameter<Eigen::VectorXd>(
      cfg["wbc"]["task"]["foot_pos"], gain_prefix + "kp");
  lfoot_pos_task->kd = util::ReadParameter<Eigen::VectorXd>(
      cfg["wbc"]["task"]["foot_pos"], gain_prefix + "kd");
  lfoot_pos_task->w_hierarchy = util::ReadParameter<double>(
      cfg["wbc"]["task"]["foot_pos"], "weight_at_swing");

  lfoot_ori_task = new LinkOriTask(robot_, {"l_foot_contact"});
  lfoot_ori_task->kp = util::ReadParameter<Eigen::VectorXd>(
      cfg["wbc"]["task"]["foot_ori"], gain_prefix + "kp");
  lfoot_ori_task->kd = util::ReadParameter<Eigen::VectorXd>(
      cfg["wbc"]["task"]["foot_ori"], gain_prefix + "kd");
  lfoot_ori_task->w_hierarchy = util::ReadParameter<double>(
      cfg["wbc"]["task"]["foot_ori"], "weight_at_swing");

  task_list.clear();
  task_list.push_back(com_task);        // 0
  task_list.push_back(cam_task);        // 1
  task_list.push_back(torso_ori_task);  // 2
  task_list.push_back(upper_body_task); // 3
  task_list.push_back(rfoot_pos_task);  // 4
  task_list.push_back(rfoot_ori_task);  // 5
  task_list.push_back(lfoot_pos_task);  // 6
  task_list.push_back(lfoot_ori_task);  // 7

  // Initialize Contact
  double foot_half_width =
      util::ReadParameter<double>(cfg["wbc"]["contact"], "foot_half_width");
  double foot_half_length =
      util::ReadParameter<double>(cfg["wbc"]["contact"], "foot_half_length");
  double mu = util::ReadParameter<double>(cfg["wbc"]["contact"], "mu");
  rfoot_contact = new SurfaceContact(robot_, "r_foot_contact", foot_half_length,
                                     foot_half_width, mu);
  rfoot_contact->rf_z_max = 1e-3;
  lfoot_contact = new SurfaceContact(robot_, "l_foot_contact", foot_half_length,
                                     foot_half_width, mu);
  lfoot_contact->rf_z_max = 1e-3;

  contact_list.clear();
  contact_list.push_back(rfoot_contact);
  contact_list.push_back(lfoot_contact);

  // Initialize Internal Constraint
  rolling_joint_constraint = new DracoRollingJointConstraint(robot_);

  internal_constraint_list.clear();
  internal_constraint_list.push_back(rolling_joint_constraint);
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

  delete rolling_joint_constraint;
}
