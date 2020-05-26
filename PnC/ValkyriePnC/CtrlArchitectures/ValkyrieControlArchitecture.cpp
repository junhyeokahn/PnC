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

    // Initialize states: add all states to the state machine map
    state_machines_[VALKYRIE_STATES::BALANCE] = new DoubleSupportStand(VALKYRIE_STATES::BALANCE, this, robot_);
    // Set Starting State
    state_ = VALKYRIE_STATES::BALANCE;
    prev_state_ = state_;

    _InitializeParameters();
}

ValkyrieControlArchitecture::~ValkyrieControlArchitecture() { 
    delete main_controller_;
    delete taf_container_;
    delete dcm_planner_;

    // Delete the trajectory managers
    delete rfoot_trajectory_manager_;

    // Delete the state machines
    delete state_machines_[VALKYRIE_STATES::BALANCE];
}

void ValkyrieControlArchitecture::ControlArchitectureInitialization() {
    taf_container_->paramInitialization(cfg_["task_parameters"]);
    main_controller_->ctrlInitialization(cfg_["controller_parameters"]);
    dcm_planner_->paramInitialization(cfg_["dcm_planner_parameters"]);
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
    try {
        double temp;
        Eigen::VectorXd temp_vec;

        YAML::Node test_cfg = cfg_["test_configuration"];
        myUtils::readParameter(test_cfg,"target_pos_duration",temp);
       ((DoubleSupportStand*) state_machines_[VALKYRIE_STATES::BALANCE])->setDuration(temp);
        myUtils::readParameter(test_cfg,"com_pos_deviation",temp_vec);
       ((DoubleSupportStand*) state_machines_[VALKYRIE_STATES::BALANCE])->setComDeviation(temp_vec);

    } catch(std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
        exit(0);
    }
}
