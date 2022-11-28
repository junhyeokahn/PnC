#include "pnc/atlas_pnc/atlas_control_architecture.hpp"

#include "pnc/atlas_pnc/atlas_controller.hpp"
#include "pnc/atlas_pnc/atlas_state_machine/contact_transition_end.hpp"
#include "pnc/atlas_pnc/atlas_state_machine/contact_transition_start.hpp"
#include "pnc/atlas_pnc/atlas_state_machine/double_support_balance.hpp"
#include "pnc/atlas_pnc/atlas_state_machine/double_support_stand.hpp"
#include "pnc/atlas_pnc/atlas_state_machine/single_support_swing.hpp"
#include "pnc/atlas_pnc/atlas_state_provider.hpp"
#include "pnc/atlas_pnc/atlas_tci_container.hpp"
#include "pnc/planners/locomotion/dcm_planner/dcm_planner.hpp"
#include "pnc/whole_body_controllers/managers/dcm_trajectory_manager.hpp"
#include "pnc/whole_body_controllers/managers/floating_base_trajectory_manager.hpp"
#include "pnc/whole_body_controllers/managers/foot_trajectory_manager.hpp"
#include "pnc/whole_body_controllers/managers/reaction_force_manager.hpp"

AtlasControlArchitecture::AtlasControlArchitecture(RobotSystem *_robot)
    : ControlArchitecture(_robot) {
  util::PrettyConstructor(1, "AtlasControlArchitecture");
  robot_ = _robot;

  // Initialize Task Force Container
  tci_container_ = new AtlasTCIContainer(robot_);

  // Initialize Controller
  controller_ = new AtlasController(tci_container_, robot_);

  // Initialize Planner
  dcm_planner_ = new DCMPlanner();

  // Initialize Task Manager
  YAML::Node cfg = YAML::LoadFile(THIS_COM "config/atlas/pnc.yaml");

  rfoot_tm = new FootTrajectoryManager(tci_container_->rfoot_pos_task,
                                       tci_container_->rfoot_ori_task, robot_);
  rfoot_tm->swing_height =
      util::ReadParameter<double>(cfg["walking"], "swing_height");
  lfoot_tm = new FootTrajectoryManager(tci_container_->lfoot_pos_task,
                                       tci_container_->lfoot_ori_task, robot_);
  lfoot_tm->swing_height =
      util::ReadParameter<double>(cfg["walking"], "swing_height");
  upper_body_tm =
      new UpperBodyTrajectoryManager(tci_container_->upper_body_task, robot_);
  floating_base_tm = new FloatingBaseTrajectoryManager(
      tci_container_->com_task, tci_container_->pelvis_ori_task, robot_);
  dcm_tm = new DCMTrajectoryManager(dcm_planner_, tci_container_->com_task,
                                    tci_container_->pelvis_ori_task, robot_,
                                    "l_sole", "r_sole");
  dcm_tm->paramInitialization(cfg["walking"]);

  // Initialize Hierarchy Manager
  Eigen::VectorXd w_contact_foot, w_swing_foot;
  util::ReadParameter(cfg["wbc"], "w_contact_foot", w_contact_foot);
  util::ReadParameter(cfg["wbc"], "w_swing_foot", w_swing_foot);
  rfoot_pos_hm = new TaskHierarchyManager(tci_container_->rfoot_pos_task,
                                          w_contact_foot, w_swing_foot);
  rfoot_ori_hm = new TaskHierarchyManager(tci_container_->rfoot_ori_task,
                                          w_contact_foot, w_swing_foot);
  lfoot_pos_hm = new TaskHierarchyManager(tci_container_->lfoot_pos_task,
                                          w_contact_foot, w_swing_foot);
  lfoot_ori_hm = new TaskHierarchyManager(tci_container_->lfoot_ori_task,
                                          w_contact_foot, w_swing_foot);

  // Initialize Reaction Force Manager
  double rf_max;
  util::ReadParameter(cfg["wbc"], "rf_z_max", rf_max);
  rfoot_fm =
      new ReactionForceManager(tci_container_->rfoot_contact, rf_max, false);
  lfoot_fm =
      new ReactionForceManager(tci_container_->lfoot_contact, rf_max, false);

  // Initialize State Machine
  state_machines[AtlasStates::Stand] =
      new DoubleSupportStand(AtlasStates::Stand, this, robot_);
  ((DoubleSupportStand *)state_machines[AtlasStates::Stand])->end_time =
      util::ReadParameter<double>(cfg["walking"], "ini_stand_dur");
  ((DoubleSupportStand *)state_machines[AtlasStates::Stand])->rf_z_max_time =
      util::ReadParameter<double>(cfg["walking"], "rf_z_max_time");
  ((DoubleSupportStand *)state_machines[AtlasStates::Stand])->com_height_des =
      util::ReadParameter<double>(cfg["walking"], "com_height");

  state_machines[AtlasStates::Balance] =
      new DoubleSupportBalance(AtlasStates::Balance, this, robot_);

  state_machines[AtlasStates::LFootContactTransitionStart] =
      new ContactTransitionStart(AtlasStates::LFootContactTransitionStart, this,
                                 EndEffector::LFoot, robot_);
  state_machines[AtlasStates::LFootContactTransitionEnd] =
      new ContactTransitionEnd(AtlasStates::LFootContactTransitionEnd, this,
                               EndEffector::LFoot, robot_);
  state_machines[AtlasStates::LFootSwing] = new SingleSupportSwing(
      AtlasStates::LFootSwing, this, EndEffector::LFoot, robot_);

  state_machines[AtlasStates::RFootContactTransitionStart] =
      new ContactTransitionStart(AtlasStates::RFootContactTransitionStart, this,
                                 EndEffector::RFoot, robot_);
  state_machines[AtlasStates::RFootContactTransitionEnd] =
      new ContactTransitionEnd(AtlasStates::RFootContactTransitionEnd, this,
                               EndEffector::RFoot, robot_);
  state_machines[AtlasStates::RFootSwing] = new SingleSupportSwing(
      AtlasStates::RFootSwing, this, EndEffector::RFoot, robot_);

  state = AtlasStates::Stand;
  prev_state = AtlasStates::Stand;

  b_state_first_visit_ = true;
  sp_ = AtlasStateProvider::getStateProvider(robot_);
}

AtlasControlArchitecture::~AtlasControlArchitecture() {
  delete tci_container_;
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

  delete state_machines[AtlasStates::Stand];
  delete state_machines[AtlasStates::Balance];
  delete state_machines[AtlasStates::LFootContactTransitionStart];
  delete state_machines[AtlasStates::LFootContactTransitionEnd];
  delete state_machines[AtlasStates::LFootSwing];
  delete state_machines[AtlasStates::RFootContactTransitionStart];
  delete state_machines[AtlasStates::RFootContactTransitionEnd];
  delete state_machines[AtlasStates::RFootSwing];
}

void AtlasControlArchitecture::getCommand(void *_command) {
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
