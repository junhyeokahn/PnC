#include "pnc/draco_pnc/draco_control_architecture.hpp"

#include "pnc/draco_pnc/draco_controller.hpp"
#include "pnc/draco_pnc/draco_state_machine/contact_transition_end.hpp"
#include "pnc/draco_pnc/draco_state_machine/contact_transition_start.hpp"
#include "pnc/draco_pnc/draco_state_machine/double_support_balance.hpp"
#include "pnc/draco_pnc/draco_state_machine/double_support_interpolation.hpp"
#include "pnc/draco_pnc/draco_state_machine/double_support_move.hpp"
#include "pnc/draco_pnc/draco_state_machine/double_support_stand.hpp"
#include "pnc/draco_pnc/draco_state_machine/double_support_swaying.hpp"
#include "pnc/draco_pnc/draco_state_machine/foot_landing.hpp"
#include "pnc/draco_pnc/draco_state_machine/foot_lifting.hpp"
#include "pnc/draco_pnc/draco_state_machine/foot_swing.hpp"
#include "pnc/draco_pnc/draco_state_machine/initialize.hpp"
#include "pnc/draco_pnc/draco_state_machine/single_support_swing.hpp"
#include "pnc/draco_pnc/draco_state_provider.hpp"
#include "pnc/draco_pnc/draco_task/draco_com_task.hpp"
#include "pnc/draco_pnc/draco_tci_container.hpp"
#include "pnc/planners/locomotion/dcm_planner/dcm_planner.hpp"
#include "pnc/whole_body_controllers/managers/dcm_trajectory_manager.hpp"
#include "pnc/whole_body_controllers/managers/floating_base_trajectory_manager.hpp"
#include "pnc/whole_body_controllers/managers/foot_trajectory_manager.hpp"
#include "pnc/whole_body_controllers/managers/reaction_force_manager.hpp"

DracoControlArchitecture::DracoControlArchitecture(RobotSystem *_robot)
    : ControlArchitecture(_robot) {
  util::PrettyConstructor(1, "DracoControlArchitecture");

  YAML::Node cfg = YAML::LoadFile(THIS_COM "config/draco/pnc.yaml");

  robot_ = _robot;
  sp_ = DracoStateProvider::getStateProvider();

  // Initialize Task Force Container
  tci_container = new DracoTCIContainer(robot_);

  // Initialize Controller
  controller_ = new DracoController(tci_container, robot_);

  // Initialize Planner
  dcm_planner_ = new DCMPlanner();

  // Initialize Trajectory Manager
  rfoot_tm = new FootTrajectoryManager(tci_container->rfoot_pos_task,
                                       tci_container->rfoot_ori_task, robot_);
  rfoot_tm->swing_height =
      util::ReadParameter<double>(cfg["walking"], "swing_height");
  lfoot_tm = new FootTrajectoryManager(tci_container->lfoot_pos_task,
                                       tci_container->lfoot_ori_task, robot_);
  lfoot_tm->swing_height =
      util::ReadParameter<double>(cfg["walking"], "swing_height");
  upper_body_tm =
      new UpperBodyTrajectoryManager(tci_container->upper_body_task, robot_);

  int com_control_feedback_height_target = util::ReadParameter<int>(
      cfg["wbc"]["task"], "com_control_feedback_height_target");
  bool b_use_base_height = false;
  if (com_control_feedback_height_target == 1) {
    b_use_base_height = true;
  }
  floating_base_tm = new FloatingBaseTrajectoryManager(
      tci_container->com_task, tci_container->torso_ori_task, robot_,
      b_use_base_height);
  dcm_tm = new DCMTrajectoryManager(dcm_planner_, tci_container->com_task,
                                    tci_container->torso_ori_task, robot_,
                                    "l_foot_contact", "r_foot_contact");
  dcm_tm->paramInitialization(cfg["walking"]);

  // Initialize Hierarchy Manager
  rfoot_pos_hm = new TaskHierarchyManager(
      tci_container->rfoot_pos_task,
      util::ReadParameter<double>(cfg["wbc"]["task"]["foot_pos"],
                                  "weight_at_contact"),
      util::ReadParameter<double>(cfg["wbc"]["task"]["foot_pos"],
                                  "weight_at_swing"));
  rfoot_ori_hm = new TaskHierarchyManager(
      tci_container->rfoot_ori_task,
      util::ReadParameter<double>(cfg["wbc"]["task"]["foot_ori"],
                                  "weight_at_contact"),
      util::ReadParameter<double>(cfg["wbc"]["task"]["foot_ori"],
                                  "weight_at_swing"));
  lfoot_pos_hm = new TaskHierarchyManager(
      tci_container->lfoot_pos_task,
      util::ReadParameter<double>(cfg["wbc"]["task"]["foot_pos"],
                                  "weight_at_contact"),
      util::ReadParameter<double>(cfg["wbc"]["task"]["foot_pos"],
                                  "weight_at_swing"));
  lfoot_ori_hm = new TaskHierarchyManager(
      tci_container->lfoot_ori_task,
      util::ReadParameter<double>(cfg["wbc"]["task"]["foot_ori"],
                                  "weight_at_contact"),
      util::ReadParameter<double>(cfg["wbc"]["task"]["foot_ori"],
                                  "weight_at_swing"));

  // Initialize Reaction Force Manager
  double rf_max;
  util::ReadParameter(cfg["wbc"]["contact"], "rf_z_max", rf_max);
  rfoot_fm = new ReactionForceManager(tci_container->rfoot_contact, rf_max);
  lfoot_fm = new ReactionForceManager(tci_container->lfoot_contact, rf_max);

  // Initialize State Machine
  state_machines[draco_states::kInitialize] =
      new Initialize(draco_states::kInitialize, this, robot_);
  ((Initialize *)state_machines[draco_states::kInitialize])->end_time =
      util::ReadParameter<double>(cfg["walking"], "ini_joint_dur");
  YAML::Node ini_jpos_node = cfg["walking"]["ini_joint_pos"];
  std::map<std::string, double> target_ini_jpos_map;
  for (const auto &kv : ini_jpos_node) {
    target_ini_jpos_map[kv.first.as<std::string>()] = kv.second.as<double>();
  }
  Eigen::VectorXd target_ini_jpos = robot_->map_to_vector(target_ini_jpos_map);
  sp_->nominal_joint_pos = target_ini_jpos_map;
  ((Initialize *)state_machines[draco_states::kInitialize])->target_jpos =
      target_ini_jpos;

  state_machines[draco_states::kStand] =
      new DoubleSupportStand(draco_states::kStand, this, robot_);
  ((DoubleSupportStand *)state_machines[draco_states::kStand])->end_time =
      util::ReadParameter<double>(cfg["walking"], "ini_stand_dur");
  ((DoubleSupportStand *)state_machines[draco_states::kStand])->rf_z_max_time =
      util::ReadParameter<double>(cfg["walking"], "rf_z_max_time");
  ((DoubleSupportStand *)state_machines[draco_states::kStand])->com_height_des =
      util::ReadParameter<double>(cfg["walking"], "com_height");

  state_machines[draco_states::kBalance] =
      new DoubleSupportBalance(draco_states::kBalance, this, robot_);

  state_machines[draco_states::kLFootContactTransitionStart] =
      new ContactTransitionStart(draco_states::kLFootContactTransitionStart,
                                 this, EndEffector::LFoot, robot_);
  state_machines[draco_states::kLFootContactTransitionEnd] =
      new ContactTransitionEnd(draco_states::kLFootContactTransitionEnd, this,
                               EndEffector::LFoot, robot_);
  state_machines[draco_states::kLFootSwing] = new SingleSupportSwing(
      draco_states::kLFootSwing, this, EndEffector::LFoot, robot_);
  ((SingleSupportSwing *)state_machines[draco_states::kLFootSwing])
      ->b_early_termination =
      util::ReadParameter<bool>(cfg["walking"], "b_early_termination");
  ((SingleSupportSwing *)state_machines[draco_states::kLFootSwing])
      ->foot_height_threshold =
      util::ReadParameter<double>(cfg["walking"], "foot_height_threshold");

  state_machines[draco_states::kRFootContactTransitionStart] =
      new ContactTransitionStart(draco_states::kRFootContactTransitionStart,
                                 this, EndEffector::RFoot, robot_);
  state_machines[draco_states::kRFootContactTransitionEnd] =
      new ContactTransitionEnd(draco_states::kRFootContactTransitionEnd, this,
                               EndEffector::RFoot, robot_);
  state_machines[draco_states::kRFootSwing] = new SingleSupportSwing(
      draco_states::kRFootSwing, this, EndEffector::RFoot, robot_);
  ((SingleSupportSwing *)state_machines[draco_states::kRFootSwing])
      ->b_early_termination =
      util::ReadParameter<bool>(cfg["walking"], "b_early_termination");
  ((SingleSupportSwing *)state_machines[draco_states::kRFootSwing])
      ->foot_height_threshold =
      util::ReadParameter<double>(cfg["walking"], "foot_height_threshold");

  state_machines[draco_states::kSwaying] =
      new DoubleSupportSwaying(draco_states::kSwaying, this, robot_);
  ((DoubleSupportSwaying *)state_machines[draco_states::kSwaying])->amp =
      util::ReadParameter<Eigen::Vector3d>(cfg["balancing"], "swaying_amp");
  ((DoubleSupportSwaying *)state_machines[draco_states::kSwaying])->freq =
      util::ReadParameter<Eigen::Vector3d>(cfg["balancing"], "swaying_freq");

  state_machines[draco_states::kBaseInterpolation] =
      new DoubleSupportInterpolation(draco_states::kBaseInterpolation, this,
                                     robot_);
  ((DoubleSupportInterpolation *)
       state_machines[draco_states::kBaseInterpolation])
      ->local_offset = util::ReadParameter<Eigen::Vector3d>(
      cfg["balancing"], "interpolation_local_offset");
  ((DoubleSupportInterpolation *)
       state_machines[draco_states::kBaseInterpolation])
      ->end_time =
      util::ReadParameter<double>(cfg["balancing"], "interpolation_duration");

  // For Static Walking
  state_machines[draco_states::kMoveCoMToLFoot] = new DoubleSupportMove(
      draco_states::kMoveCoMToLFoot, this, com_move_states::Left, robot_);
  ((DoubleSupportMove *)state_machines[draco_states::kMoveCoMToLFoot])
      ->moving_duration_ =
      util::ReadParameter<double>(cfg["static_walking"], "moving_duration");
  ((DoubleSupportMove *)state_machines[draco_states::kMoveCoMToLFoot])
      ->des_com_height_ =
      util::ReadParameter<double>(cfg["static_walking"], "com_height");
  ((DoubleSupportMove *)state_machines[draco_states::kMoveCoMToLFoot])
      ->com_offset =
      util::ReadParameter<Eigen::Vector2d>(cfg["static_walking"], "com_offset");

  state_machines[draco_states::kMoveCoMToRFoot] = new DoubleSupportMove(
      draco_states::kMoveCoMToRFoot, this, com_move_states::Right, robot_);
  ((DoubleSupportMove *)state_machines[draco_states::kMoveCoMToRFoot])
      ->moving_duration_ =
      util::ReadParameter<double>(cfg["static_walking"], "moving_duration");
  ((DoubleSupportMove *)state_machines[draco_states::kMoveCoMToRFoot])
      ->des_com_height_ =
      util::ReadParameter<double>(cfg["static_walking"], "com_height");

  state_machines[draco_states::kMoveCoMToCenter] = new DoubleSupportMove(
      draco_states::kMoveCoMToCenter, this, com_move_states::Center, robot_);
  ((DoubleSupportMove *)state_machines[draco_states::kMoveCoMToCenter])
      ->moving_duration_ =
      util::ReadParameter<double>(cfg["static_walking"], "moving_duration");
  ((DoubleSupportMove *)state_machines[draco_states::kMoveCoMToCenter])
      ->des_com_height_ =
      util::ReadParameter<double>(cfg["static_walking"], "com_height");

  state_machines[draco_states::kLFootLifting] = new FootLifting(
      draco_states::kLFootSwing, this, EndEffector::LFoot, robot_);

  state_machines[draco_states::kRFootLifting] = new FootLifting(
      draco_states::kRFootSwing, this, EndEffector::RFoot, robot_);

  state_machines[draco_states::kLFootSwingStatic] = new FootSwing(
      draco_states::kLFootSwingStatic, this, EndEffector::LFoot, robot_);
  ((FootSwing *)state_machines[draco_states::kLFootSwingStatic])
      ->swing_duration_ =
      util::ReadParameter<double>(cfg["static_walking"], "swing_duration");
  ((FootSwing *)state_machines[draco_states::kLFootSwingStatic])
      ->des_foot_x_increment_ = util::ReadParameter<double>(
      cfg["static_walking"], "des_foot_x_inc_local");
  ((FootSwing *)state_machines[draco_states::kLFootSwingStatic])
      ->des_foot_y_increment_ = util::ReadParameter<double>(
      cfg["static_walking"], "des_foot_y_inc_local");
  ((FootSwing *)state_machines[draco_states::kLFootSwingStatic])
      ->des_foot_ori_increment_ = util::ReadParameter<double>(
      cfg["static_walking"], "des_foot_ori_inc_local");

  state_machines[draco_states::kRFootSwingStatic] = new FootSwing(
      draco_states::kRFootSwingStatic, this, EndEffector::RFoot, robot_);
  ((FootSwing *)state_machines[draco_states::kRFootSwingStatic])
      ->swing_duration_ =
      util::ReadParameter<double>(cfg["static_walking"], "swing_duration");
  ((FootSwing *)state_machines[draco_states::kRFootSwingStatic])
      ->des_foot_x_increment_ = util::ReadParameter<double>(
      cfg["static_walking"], "des_foot_x_inc_local");
  ((FootSwing *)state_machines[draco_states::kRFootSwingStatic])
      ->des_foot_y_increment_ = util::ReadParameter<double>(
      cfg["static_walking"], "des_foot_y_inc_local");
  ((FootSwing *)state_machines[draco_states::kRFootSwingStatic])
      ->des_foot_ori_increment_ = util::ReadParameter<double>(
      cfg["static_walking"], "des_foot_ori_inc_local");

  state_machines[draco_states::kLFootLanding] = new FootLanding(
      draco_states::kLFootLanding, this, EndEffector::LFoot, robot_);

  state_machines[draco_states::kRFootLanding] = new FootLanding(
      draco_states::kRFootLanding, this, EndEffector::RFoot, robot_);

  state = draco_states::kStand;
  prev_state = draco_states::kStand;

  b_state_first_visit_ = true;
}

DracoControlArchitecture::~DracoControlArchitecture() {
  delete tci_container;

  delete controller_;
  delete dcm_planner_;

  delete rfoot_tm;
  delete lfoot_tm;
  delete upper_body_tm;
  delete floating_base_tm;
  delete dcm_tm;

  delete rfoot_pos_hm;
  delete rfoot_ori_hm;
  delete lfoot_pos_hm;
  delete lfoot_ori_hm;

  delete rfoot_fm;
  delete lfoot_fm;

  delete state_machines[draco_states::kInitialize];
  delete state_machines[draco_states::kStand];
  delete state_machines[draco_states::kBalance];
  delete state_machines[draco_states::kLFootContactTransitionStart];
  delete state_machines[draco_states::kLFootContactTransitionEnd];
  delete state_machines[draco_states::kLFootSwing];
  delete state_machines[draco_states::kRFootContactTransitionStart];
  delete state_machines[draco_states::kRFootContactTransitionEnd];
  delete state_machines[draco_states::kRFootSwing];
  delete state_machines[draco_states::kSwaying];
  delete state_machines[draco_states::kBaseInterpolation];
  delete state_machines[draco_states::kMoveCoMToLFoot];
  delete state_machines[draco_states::kMoveCoMToRFoot];
  delete state_machines[draco_states::kMoveCoMToCenter];
  delete state_machines[draco_states::kRFootLifting];
  delete state_machines[draco_states::kLFootLifting];
  delete state_machines[draco_states::kRFootSwingStatic];
  delete state_machines[draco_states::kLFootSwingStatic];
  delete state_machines[draco_states::kRFootLanding];
  delete state_machines[draco_states::kLFootLanding];
}

void DracoControlArchitecture::getCommand(void *_command) {
  // Initialize Staet
  if (b_state_first_visit_) {
    state_machines[state]->firstVisit();
    b_state_first_visit_ = false;
  }

  // Update State Machine
  state_machines[state]->oneStep();
  // Update State Machine Independent Trajectories
  upper_body_tm->UseNominalUpperBodyJointPos(sp_->nominal_joint_pos);
  // Get WBC Commands
  controller_->getCommand(_command);

  if (state_machines[state]->endOfState()) {
    state_machines[state]->lastVisit();
    prev_state = state;
    state = state_machines[state]->getNextState();
    b_state_first_visit_ = true;
  }

  if (sp_->count % sp_->save_freq == 0) {
    this->SaveData();
  }
}

void DracoControlArchitecture::SaveData() {
  DracoDataManager *dm = DracoDataManager::GetDracoDataManager();

  tci_container->com_task->CopyData(
      dm->data->task_com_pos_des, dm->data->task_com_vel_des,
      dm->data->task_com_acc_des, dm->data->task_com_pos,
      dm->data->task_com_vel, dm->data->task_com_pos_des_local,
      dm->data->task_com_vel_des_local, dm->data->task_com_acc_des_local,
      dm->data->task_com_pos_local, dm->data->task_com_vel_local);

  dm->data->icp_des =
      dynamic_cast<DracoCenterOfMassTask *>(tci_container->com_task)->icp_des;
  dm->data->icp_dot_des =
      dynamic_cast<DracoCenterOfMassTask *>(tci_container->com_task)
          ->icp_dot_des;

  Eigen::VectorXd dummy = Eigen::VectorXd::Zero(3);
  tci_container->cam_task->CopyData(
      dummy, dm->data->task_cam_vel_des, dm->data->task_cam_acc_des, dummy,
      dm->data->task_cam_vel, dummy, dm->data->task_cam_vel_des_local,
      dm->data->task_cam_acc_des_local, dummy, dm->data->task_cam_vel_local);

  tci_container->torso_ori_task->CopyData(
      dm->data->task_torso_ori_pos_des, dm->data->task_torso_ori_vel_des,
      dm->data->task_torso_ori_acc_des, dm->data->task_torso_ori_pos,
      dm->data->task_torso_ori_vel, dm->data->task_torso_ori_pos_des_local,
      dm->data->task_torso_ori_vel_des_local,
      dm->data->task_torso_ori_acc_des_local,
      dm->data->task_torso_ori_pos_local, dm->data->task_torso_ori_vel_local);

  tci_container->rfoot_pos_task->CopyData(
      dm->data->task_rfoot_lin_pos_des, dm->data->task_rfoot_lin_vel_des,
      dm->data->task_rfoot_lin_acc_des, dm->data->task_rfoot_lin_pos,
      dm->data->task_rfoot_lin_vel, dm->data->task_rfoot_lin_pos_des_local,
      dm->data->task_rfoot_lin_vel_des_local,
      dm->data->task_rfoot_lin_acc_des_local,
      dm->data->task_rfoot_lin_pos_local, dm->data->task_rfoot_lin_vel_local);

  tci_container->rfoot_ori_task->CopyData(
      dm->data->task_rfoot_ori_pos_des, dm->data->task_rfoot_ori_vel_des,
      dm->data->task_rfoot_ori_acc_des, dm->data->task_rfoot_ori_pos,
      dm->data->task_rfoot_ori_vel, dm->data->task_rfoot_ori_pos_des_local,
      dm->data->task_rfoot_ori_vel_des_local,
      dm->data->task_rfoot_ori_acc_des_local,
      dm->data->task_rfoot_ori_pos_local, dm->data->task_rfoot_ori_vel_local);

  tci_container->lfoot_pos_task->CopyData(
      dm->data->task_lfoot_lin_pos_des, dm->data->task_lfoot_lin_vel_des,
      dm->data->task_lfoot_lin_acc_des, dm->data->task_lfoot_lin_pos,
      dm->data->task_lfoot_lin_vel, dm->data->task_lfoot_lin_pos_des_local,
      dm->data->task_lfoot_lin_vel_des_local,
      dm->data->task_lfoot_lin_acc_des_local,
      dm->data->task_lfoot_lin_pos_local, dm->data->task_lfoot_lin_vel_local);

  tci_container->lfoot_ori_task->CopyData(
      dm->data->task_lfoot_ori_pos_des, dm->data->task_lfoot_ori_vel_des,
      dm->data->task_lfoot_ori_acc_des, dm->data->task_lfoot_ori_pos,
      dm->data->task_lfoot_ori_vel, dm->data->task_lfoot_ori_pos_des_local,
      dm->data->task_lfoot_ori_vel_des_local,
      dm->data->task_lfoot_ori_acc_des_local,
      dm->data->task_lfoot_ori_pos_local, dm->data->task_lfoot_ori_vel_local);

  tci_container->upper_body_task->CopyData(
      dm->data->task_upper_body_pos_des, dm->data->task_upper_body_vel_des,
      dm->data->task_upper_body_acc_des, dm->data->task_upper_body_pos,
      dm->data->task_upper_body_vel);
}
