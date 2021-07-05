#include "PnC/draco_pnc/draco_control_architecture.hpp"

#include "PnC/Planner/Locomotion/DCMPlanner/DCMPlanner.hpp"
#include "PnC/WBC/Manager/DCMTrajectoryManager.hpp"
#include "PnC/WBC/Manager/FloatingBaseTrajectoryManager.hpp"
#include "PnC/WBC/Manager/FootSE3TrajectoryManager.hpp"
#include "PnC/WBC/Manager/ReactionForceManager.hpp"
#include "PnC/draco_pnc/draco_controller.hpp"
#include "PnC/draco_pnc/draco_state_machine/contact_transition_end.hpp"
#include "PnC/draco_pnc/draco_state_machine/contact_transition_start.hpp"
#include "PnC/draco_pnc/draco_state_machine/double_support_balance.hpp"
#include "PnC/draco_pnc/draco_state_machine/double_support_stand.hpp"
#include "PnC/draco_pnc/draco_state_machine/double_support_swaying.hpp"
#include "PnC/draco_pnc/draco_state_machine/initialize.hpp"
#include "PnC/draco_pnc/draco_state_machine/single_support_swing.hpp"
#include "PnC/draco_pnc/draco_state_provider.hpp"
#include "PnC/draco_pnc/draco_tci_container.hpp"

DracoControlArchitecture::DracoControlArchitecture(RobotSystem *_robot)
    : ControlArchitecture(_robot) {
  robot_ = _robot;

  // Initialize Task Force Container
  tci_container = new DracoTCIContainer(robot_);

  // Initialize Controller
  controller_ = new DracoController(tci_container, robot_);

  // Initialize Planner
  dcm_planner_ = new DCMPlanner();

  // Initialize Task Manager
  YAML::Node cfg = YAML::LoadFile(THIS_COM "Config/draco/pnc.yaml");

  rfoot_tm = new FootSE3TrajectoryManager(
      tci_container->rfoot_pos_task, tci_container->rfoot_ori_task, robot_);
  rfoot_tm->swing_height =
      util::ReadParameter<double>(cfg["walking"], "swing_height");
  lfoot_tm = new FootSE3TrajectoryManager(
      tci_container->lfoot_pos_task, tci_container->lfoot_ori_task, robot_);
  lfoot_tm->swing_height =
      util::ReadParameter<double>(cfg["walking"], "swing_height");
  upper_body_tm =
      new UpperBodyTrajectoryManager(tci_container->upper_body_task, robot_);
  floating_base_tm = new FloatingBaseTrajectoryManager(
      tci_container->com_task, tci_container->torso_ori_task, robot_);
  dcm_tm = new DCMTrajectoryManager(dcm_planner_, tci_container->com_task,
                                    tci_container->torso_ori_task, robot_,
                                    "l_foot_contact", "r_foot_contact");
  dcm_tm->paramInitialization(cfg["walking"]);

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
  double rf_max;
  util::ReadParameter(cfg["wbc"], "rf_z_max", rf_max);
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

  state_machines[draco_states::kRFootContactTransitionStart] =
      new ContactTransitionStart(draco_states::kRFootContactTransitionStart,
                                 this, EndEffector::RFoot, robot_);
  state_machines[draco_states::kRFootContactTransitionEnd] =
      new ContactTransitionEnd(draco_states::kRFootContactTransitionEnd, this,
                               EndEffector::RFoot, robot_);
  state_machines[draco_states::kRFootSwing] = new SingleSupportSwing(
      draco_states::kRFootSwing, this, EndEffector::RFoot, robot_);

  state_machines[draco_states::kSwaying] =
      new DoubleSupportSwaying(draco_states::kSwaying, this, robot_);
  ((DoubleSupportSwaying *)state_machines[draco_states::kSwaying])->amp =
      util::ReadParameter<Eigen::Vector3d>(cfg["walking"], "swaying_amp");
  ((DoubleSupportSwaying *)state_machines[draco_states::kSwaying])->freq =
      util::ReadParameter<Eigen::Vector3d>(cfg["walking"], "swaying_freq");

  state = draco_states::kStand;
  prev_state = draco_states::kStand;

  b_state_first_visit_ = true;
  sp_ = DracoStateProvider::getStateProvider();
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
}
