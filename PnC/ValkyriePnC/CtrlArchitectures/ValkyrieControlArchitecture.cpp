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

    lfoot_max_normal_force_manager_ = new MaxNormalForceTrajectoryManager(taf_container_->lfoot_contact_, robot_);
    rfoot_max_normal_force_manager_ = new MaxNormalForceTrajectoryManager(taf_container_->rfoot_contact_, robot_);
    dcm_trajectory_manger_ = new DCMPlannerTrajectoryManager(dcm_planner_, robot_);


    // Initialize states: add all states to the state machine map
    state_machines_[VALKYRIE_STATES::STAND] = new DoubleSupportStand(VALKYRIE_STATES::BALANCE, this, robot_);
    state_machines_[VALKYRIE_STATES::BALANCE] = new DoubleSupportBalance(VALKYRIE_STATES::BALANCE, this, robot_);
    state_machines_[VALKYRIE_STATES::INITIAL_TRANSFER] = new InitialTransfer(VALKYRIE_STATES::INITIAL_TRANSFER, this, robot_);    
    // Set Starting State
    state_ = VALKYRIE_STATES::STAND;
    prev_state_ = state_;

    _InitializeParameters();
}

ValkyrieControlArchitecture::~ValkyrieControlArchitecture() { 
    delete main_controller_;
    delete taf_container_;
    delete dcm_planner_;

    // Delete the trajectory managers
    delete dcm_trajectory_manger_;
    delete rfoot_trajectory_manager_;
    delete lfoot_trajectory_manager_;

    // Delete the state machines
    delete state_machines_[VALKYRIE_STATES::STAND];
    delete state_machines_[VALKYRIE_STATES::BALANCE];
    delete state_machines_[VALKYRIE_STATES::INITIAL_TRANSFER];

}

void ValkyrieControlArchitecture::ControlArchitectureInitialization() {
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
    dcm_planner_->paramInitialization(cfg_["dcm_planner_parameters"]);

    // States Initialization:
    state_machines_[VALKYRIE_STATES::STAND]->initialization(cfg_["state_stand_params"]);

}
