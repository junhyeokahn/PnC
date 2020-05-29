#include <PnC/ValkyriePnC/CtrlArchitectures/ValkyrieControlArchitecture.hpp>

ValkyrieControlArchitecture::ValkyrieControlArchitecture(RobotSystem* _robot) : ControlArchitecture(_robot) {
    b_state_first_visit_ = true;

    myUtils::pretty_constructor(1, "Valkyrie Control Architecture");
    cfg_ = YAML::LoadFile(THIS_COM "Config/Valkyrie/TEST/CONTROL_ARCHITECTURE_PARAMS.yaml");

    sp_ = ValkyrieStateProvider::getStateProvider(robot_);

    // Initialize Main Controller
    taf_container_ = new ValkyrieTaskAndForceContainer(robot_);
    main_controller_ = new ValkyrieMainController(taf_container_, robot_);
    // Initialize Planner
    dcm_planner_ = new DCMPlanner();

    // Initialize Trajectory managers
    rfoot_trajectory_manager_ = new FootSE3TrajectoryManager(taf_container_->rfoot_center_pos_task_, 
                                                             taf_container_->rfoot_center_ori_task_, robot_);
    lfoot_trajectory_manager_ = new FootSE3TrajectoryManager(taf_container_->lfoot_center_pos_task_, 
                                                             taf_container_->lfoot_center_ori_task_, robot_);
    rfoot_max_normal_force_manager_ = new MaxNormalForceTrajectoryManager(taf_container_->rfoot_contact_, robot_);
    lfoot_max_normal_force_manager_ = new MaxNormalForceTrajectoryManager(taf_container_->lfoot_contact_, robot_);
    dcm_trajectory_manger_ = new DCMPlannerTrajectoryManager(dcm_planner_, robot_);

    // Initialize states: add all states to the state machine map
    state_machines_[VALKYRIE_STATES::STAND] = new DoubleSupportStand(VALKYRIE_STATES::STAND, this, robot_);
    state_machines_[VALKYRIE_STATES::BALANCE] = new DoubleSupportBalance(VALKYRIE_STATES::BALANCE, this, robot_);

    state_machines_[VALKYRIE_STATES::RL_CONTACT_TRANSITION_START] = new ContactTransition(VALKYRIE_STATES::RL_CONTACT_TRANSITION_START, RIGHT_ROBOT_SIDE, this, robot_);    
    state_machines_[VALKYRIE_STATES::RL_CONTACT_TRANSITION_END] = new ContactTransitionEnd(VALKYRIE_STATES::RL_CONTACT_TRANSITION_END, RIGHT_ROBOT_SIDE, this, robot_);    
    state_machines_[VALKYRIE_STATES::RL_SWING] = new SwingControl(VALKYRIE_STATES::RL_SWING, RIGHT_ROBOT_SIDE, this, robot_);    

    state_machines_[VALKYRIE_STATES::LL_CONTACT_TRANSITION_START] = new ContactTransition(VALKYRIE_STATES::LL_CONTACT_TRANSITION_START, LEFT_ROBOT_SIDE, this, robot_);    
    state_machines_[VALKYRIE_STATES::LL_CONTACT_TRANSITION_END] = new ContactTransitionEnd(VALKYRIE_STATES::LL_CONTACT_TRANSITION_END, LEFT_ROBOT_SIDE, this, robot_);    
    state_machines_[VALKYRIE_STATES::LL_SWING] = new SwingControl(VALKYRIE_STATES::LL_SWING, LEFT_ROBOT_SIDE, this, robot_);    

    // Set Starting State
    state_ = VALKYRIE_STATES::STAND;
    prev_state_ = state_;

    _InitializeParameters();
}

ValkyrieControlArchitecture::~ValkyrieControlArchitecture() { 
    delete taf_container_;
    delete main_controller_;
    delete dcm_planner_;

    // Delete the trajectory managers
    delete rfoot_trajectory_manager_;
    delete lfoot_trajectory_manager_;
    delete rfoot_max_normal_force_manager_;
    delete lfoot_max_normal_force_manager_;
    delete dcm_trajectory_manger_;

    // Delete the state machines
    delete state_machines_[VALKYRIE_STATES::STAND];
    delete state_machines_[VALKYRIE_STATES::BALANCE];
    delete state_machines_[VALKYRIE_STATES::RL_CONTACT_TRANSITION_START];
    delete state_machines_[VALKYRIE_STATES::LL_CONTACT_TRANSITION_START];    
}

void ValkyrieControlArchitecture::ControlArchitectureInitialization() {
    dcm_trajectory_manger_->setCoMandPelvisTasks(taf_container_->com_task_,
                                                 taf_container_->pelvis_ori_task_);
}

void ValkyrieControlArchitecture::getCommand(void* _command) {
    // Initialize State
    if (b_state_first_visit_) {
        state_machines_[state_]->firstVisit();
        b_state_first_visit_ = false;
    }
    // Get Commands
    state_machines_[state_]->oneStep();
    main_controller_->getCommand(_command);

    // Check for State Transitions
    if (state_machines_[state_]->endOfState()) {
        state_machines_[state_]->lastVisit();
        prev_state_ = state_;
        state_ = state_machines_[state_]->getNextState();
        b_state_first_visit_ = true;
    }
};

void ValkyrieControlArchitecture::_InitializeParameters() {
    taf_container_->paramInitialization(cfg_["task_parameters"]);
    main_controller_->ctrlInitialization(cfg_["controller_parameters"]);

    // Trajectory Managers initialization
    rfoot_trajectory_manager_->paramInitialization(cfg_["foot_trajectory_parameters"]);
    lfoot_trajectory_manager_->paramInitialization(cfg_["foot_trajectory_parameters"]);
    lfoot_max_normal_force_manager_->paramInitialization(cfg_["task_parameters"]);   
    rfoot_max_normal_force_manager_->paramInitialization(cfg_["task_parameters"]);   
    dcm_trajectory_manger_->paramInitialization(cfg_["dcm_planner_parameters"]);

    // States Initialization:
    state_machines_[VALKYRIE_STATES::STAND]->initialization(cfg_["state_stand_params"]);

}
