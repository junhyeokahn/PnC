#include "pnc/fixed_draco_pnc/fixed_draco_control_architecture.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_controller.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_state_machine/end_effector_hold.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_state_machine/end_effector_swaying.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_state_machine/initialize.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_state_provider.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_tci_container.hpp"

FixedDracoControlArchitecture::FixedDracoControlArchitecture(
    RobotSystem *_robot)
    : ControlArchitecture(_robot) {
  util::PrettyConstructor(1, "FixedDracoControlArchitecture");

  YAML::Node cfg = YAML::LoadFile(THIS_COM "config/fixed_draco/pnc.yaml");

  robot_ = _robot;
  sp_ = FixedDracoStateProvider::getStateProvider();

  // Initialize Task Force Container
  tci_container = new FixedDracoTCIContainer(robot_);

  // Initialize Controller
  controller_ = new FixedDracoController(tci_container, robot_);

  // Initialize Planner

  // Initialize Trajectory Manager
  rf_ee_tm = new EndEffectorTrajectoryManager(
      tci_container->rfoot_pos_task, tci_container->rfoot_ori_task, robot_);
  lf_ee_tm = new EndEffectorTrajectoryManager(
      tci_container->lfoot_pos_task, tci_container->lfoot_ori_task, robot_);
  upper_body_tm =
      new UpperBodyTrajectoryManager(tci_container->upper_body_task, robot_);

  // Initialize State Machine
  state_machines[fixed_draco_states::kInitialize] =
      new Initialize(fixed_draco_states::kInitialize, this, robot_);
  ((Initialize *)state_machines[fixed_draco_states::kInitialize])->end_time =
      util::ReadParameter<double>(cfg["behavior"], "ini_joint_dur");
  YAML::Node ini_jpos_node = cfg["behavior"]["ini_joint_pos"];
  std::map<std::string, double> target_ini_jpos_map;
  for (const auto &kv : ini_jpos_node) {
    target_ini_jpos_map[kv.first.as<std::string>()] = kv.second.as<double>();
  }
  Eigen::VectorXd target_ini_jpos = robot_->map_to_vector(target_ini_jpos_map);
  sp_->nominal_joint_pos = target_ini_jpos_map;
  ((Initialize *)state_machines[fixed_draco_states::kInitialize])->target_jpos =
      target_ini_jpos;

  state_machines[fixed_draco_states::kHold] =
      new EndEffectorHold(fixed_draco_states::kHold, this, robot_);

  state_machines[fixed_draco_states::kRightFootSwaying] =
      new EndEffectorSwaying(fixed_draco_states::kRightFootSwaying, this,
                             EndEffector::RFoot, robot_);
  ((EndEffectorSwaying *)state_machines[fixed_draco_states::kRightFootSwaying])
      ->amp =
      util::ReadParameter<Eigen::Vector3d>(cfg["behavior"], "swaying_amp");
  ((EndEffectorSwaying *)state_machines[fixed_draco_states::kRightFootSwaying])
      ->freq =
      util::ReadParameter<Eigen::Vector3d>(cfg["behavior"], "swaying_freq");

  state_machines[fixed_draco_states::kLeftFootSwaying] = new EndEffectorSwaying(
      fixed_draco_states::kLeftFootSwaying, this, EndEffector::LFoot, robot_);
  ((EndEffectorSwaying *)state_machines[fixed_draco_states::kLeftFootSwaying])
      ->amp =
      util::ReadParameter<Eigen::Vector3d>(cfg["behavior"], "swaying_amp");
  ((EndEffectorSwaying *)state_machines[fixed_draco_states::kLeftFootSwaying])
      ->freq =
      util::ReadParameter<Eigen::Vector3d>(cfg["behavior"], "swaying_freq");

  state = fixed_draco_states::kHold;
  prev_state = fixed_draco_states::kHold;

  b_state_first_visit_ = true;
}

FixedDracoControlArchitecture::~FixedDracoControlArchitecture() {
  delete tci_container;

  delete controller_;

  delete upper_body_tm;

  delete rf_ee_tm;
  delete lf_ee_tm;

  delete state_machines[fixed_draco_states::kInitialize];
  delete state_machines[fixed_draco_states::kHold];
  delete state_machines[fixed_draco_states::kRightFootSwaying];
  delete state_machines[fixed_draco_states::kLeftFootSwaying];
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
  FixedDracoDataManager *dm = FixedDracoDataManager::GetFixedDracoDataManager();

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
