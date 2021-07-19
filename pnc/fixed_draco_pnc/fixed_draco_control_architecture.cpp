#include "pnc/fixed_draco_pnc/fixed_draco_control_architecture.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_controller.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_state_machine/contact_transition_end.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_state_machine/contact_transition_start.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_state_machine/initialize.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_state_machine/endeffector_swaying.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_state_provider.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_tci_container.hpp"
#include "pnc/whole_body_controllers/managers/foot_trajectory_manager.hpp"
#include "pnc/whole_body_controllers/managers/reaction_force_manager.hpp"

FixedDracoControlArchitecture::FixedDracoControlArchitecture(RobotSystem *_robot)
    : ControlArchitecture(_robot) {
  util::PrettyConstructor(1, "FixedDracoControlArchitecture");
  robot_ = _robot;

  // Initialize Task Force Container
  tci_container = new FixedDracoTCIContainer(robot_);

  // Initialize Controller
  controller_ = new FixedDracoController(tci_container, robot_);

  // Initialize Planner

  // Initialize Task Manager
  YAML::Node cfg = YAML::LoadFile(THIS_COM "config/fixed_draco/pnc.yaml");

  //TODO:change foot traj manager to normal ee traj manager
  //rfoot_tm = new FootTrajectoryManager(tci_container->rfoot_pos_task,
                                       //tci_container->rfoot_ori_task, robot_);
  //rfoot_tm->swing_height =
      //util::ReadParameter<double>(cfg["walking"], "swing_height");

  rf_ee_tm = new EndEffectorTrajectoryManager(tci_container->rfoot_pos_task,
                                              tci_container->rfoot_ori_task, robot_);
  lf_ee_tm = new EndEffectorTrajectoryManager(tci_container->lfoot_pos_task,
                                              tci_container->lfoot_ori_task, robot_);
  upper_body_tm =
      new UpperBodyTrajectoryManager(tci_container->upper_body_task, robot_);


  // Initialize Hierarchy Manager
  double w_contact_foot, w_swing_foot;
  util::ReadParameter(cfg["wbc"], "w_contact_foot", w_contact_foot);
  util::ReadParameter(cfg["wbc"], "w_swing_foot", w_swing_foot);
  rfoot_pos_hm = new TaskHierarchyManager(tci_container->rfoot_pos_task,
                                          w_contact_foot, w_swing_foot);
  rfoot_ori_hm = new TaskHierarchyManager(tci_container->rfoot_ori_task,
                                          w_contact_foot, w_swing_foot);
  lfoot_pos_hm = new TaskHierarchyManager(tci_container->lfoot_pos_task,
                                          w_contact_foot, w_swing_foot);
  lfoot_ori_hm = new TaskHierarchyManager(tci_container->lfoot_ori_task,
                                          w_contact_foot, w_swing_foot);
  // Initialize Reaction Force Manager

  // Initialize State Machine
  state_machines[draco_states::kInitialize] =
      new Initialize(draco_states::kInitialize, this, robot_);
  ((Initialize *)state_machines[draco_states::kInitialize])->end_time =
      util::ReadParameter<double>(cfg["ee_swing"], "ini_joint_dur");
  YAML::Node ini_jpos_node = cfg["ee_swing"]["ini_joint_pos"];
  std::map<std::string, double> target_ini_jpos_map;
  for (const auto &kv : ini_jpos_node) {
    target_ini_jpos_map[kv.first.as<std::string>()] = kv.second.as<double>();
  }
  Eigen::VectorXd target_ini_jpos = robot_->map_to_vector(target_ini_jpos_map);
  ((Initialize *)state_machines[draco_states::kInitialize])->target_jpos =
      target_ini_jpos;

  state_machines[draco_states::kSwing] =
      new EndEffectorSwaying(draco_states::kSwing, this, robot_);
  ((EndEffectorSwaying *)state_machines[draco_states::kSwing])->amp =
      util::ReadParameter<Eigen::Vector3d>(cfg["ee_swing"], "swaying_amp");
  ((EndEffectorSwaying *)state_machines[draco_states::kSwing])->freq =
      util::ReadParameter<Eigen::Vector3d>(cfg["ee_swing"], "swaying_freq");


  state = draco_states::kHold;
  prev_state = draco_states::kHold;

  b_state_first_visit_ = true;
  sp_ = DracoStateProvider::getStateProvider();
}

FixedDracoControlArchitecture::~FixedDracoControlArchitecture() {
  delete tci_container;

  delete controller_;

  delete rfoot_tm;
  delete lfoot_tm;
  delete upper_body_tm;

  delete rfoot_pos_hm;
  delete rfoot_ori_hm;

  delete state_machines[draco_states::kInitialize];
  delete state_machines[draco_states::kSwing];
}

void FixedDracoControlArchitecture::getCommand(void *_command) {
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

void FixedDracoControlArchitecture::SaveData() {
  DracoDataManager *dm = DracoDataManager::GetDracoDataManager();

  tci_container->com_task->CopyData(
      dm->data->task_com_pos_des, dm->data->task_com_vel_des,
      dm->data->task_com_acc_des, dm->data->task_com_pos,
      dm->data->task_com_vel);

  tci_container->torso_ori_task->CopyData(
      dm->data->task_torso_ori_des, dm->data->task_torso_ang_vel_des,
      dm->data->task_torso_ang_acc_des, dm->data->task_torso_ori,
      dm->data->task_torso_ang_vel);

  tci_container->rfoot_pos_task->CopyData(
      dm->data->task_rfoot_pos_des, dm->data->task_rfoot_vel_des,
      dm->data->task_rfoot_acc_des, dm->data->task_rfoot_pos,
      dm->data->task_rfoot_vel);

  tci_container->rfoot_ori_task->CopyData(
      dm->data->task_rfoot_ori_des, dm->data->task_rfoot_ang_vel_des,
      dm->data->task_rfoot_ang_acc_des, dm->data->task_rfoot_ori,
      dm->data->task_rfoot_ang_vel);

  tci_container->lfoot_pos_task->CopyData(
      dm->data->task_lfoot_pos_des, dm->data->task_lfoot_vel_des,
      dm->data->task_lfoot_acc_des, dm->data->task_lfoot_pos,
      dm->data->task_lfoot_vel);

  tci_container->lfoot_ori_task->CopyData(
      dm->data->task_lfoot_ori_des, dm->data->task_lfoot_ang_vel_des,
      dm->data->task_lfoot_ang_acc_des, dm->data->task_lfoot_ori,
      dm->data->task_lfoot_ang_vel);

  tci_container->upper_body_task->CopyData(
      dm->data->task_upper_body_pos_des, dm->data->task_upper_body_vel_des,
      dm->data->task_upper_body_acc_des, dm->data->task_upper_body_pos,
      dm->data->task_upper_body_vel);
}
