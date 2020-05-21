#include <PnC/ValkyriePnC/CtrlArchitectures/ValkyrieControlArchitecture.hpp>

ValkyrieControlArchitecture::ValkyrieControlArchitecture(RobotSystem* robot) : ControlArchitecture(robot) {
    b_first_visit_ = true;

    myUtils::pretty_constructor(1, "Valkyrie Control Architecture");
    cfg_ = YAML::LoadFile(THIS_COM "Config/Valkyrie/TEST/BALANCE_TEST.yaml");

    sp_ = ValkyrieStateProvider::getStateProvider(robot_);

    // Add all states to the state machine
    // state_machines[VALKYRIE_STATES::BALANCE] = new StateMachine(VALKYRIE_STATES::BALANCE, this, robot_);
    // Set Starting State
    state_ = VALKYRIE_STATES::BALANCE;

    balance_ctrl_ = new DCMBalanceCtrl(robot_);
    _SettingParameter();
}

ValkyrieControlArchitecture::~ValkyrieControlArchitecture() { 
    delete balance_ctrl_; 
    _DeleteController();
    _DeleteTasks();
    _DeleteContacts();
}

void ValkyrieControlArchitecture::ControlArchitectureInitialization() {
    balance_ctrl_->ctrlInitialization(
        cfg_["control_configuration"]["balance_ctrl"]);
}


void ValkyrieControlArchitecture::getCommand(void* _command) {
    if (b_first_visit_) {
        balance_ctrl_->firstVisit();
        b_first_visit_ = false;
    }
    balance_ctrl_->oneStep(_command);
};

void ValkyrieControlArchitecture::_SettingParameter() {
    try {
        double temp;
        Eigen::VectorXd temp_vec;

        YAML::Node test_cfg = cfg_["test_configuration"];
        myUtils::readParameter(test_cfg,"target_pos_duration",temp);
       ((DCMBalanceCtrl*)balance_ctrl_)->setDuration(temp);
        myUtils::readParameter(test_cfg,"com_pos_deviation",temp_vec);
       ((DCMBalanceCtrl*)balance_ctrl_)->setComDeviation(temp_vec);
        myUtils::readParameter(test_cfg,"amplitude",temp_vec);

    } catch(std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
        exit(0);
    }
}

void ValkyrieControlArchitecture::_InitializeParameters(){
}

void ValkyrieControlArchitecture::_InitializeController(){
    // Initialize IHWBC
    std::vector<bool> act_list;
    act_list.resize(Valkyrie::n_dof, true);
    for (int i(0); i < Valkyrie::n_vdof; ++i) act_list[i] = false;
    ihwbc_ = new IHWBC(act_list);

    // Initialize Joint Integrator
    ihwbc_dt_ = ValkyrieAux::servo_rate;
    ihwbc_joint_integrator_ = new IHWBC_JointIntegrator(Valkyrie::n_adof, ihwbc_dt_);

    // Initialize desired pos, vel, acc containers
    des_jpos_ = Eigen::VectorXd::Zero(Valkyrie::n_adof);
    des_jvel_ = Eigen::VectorXd::Zero(Valkyrie::n_adof);
    des_jacc_ = Eigen::VectorXd::Zero(Valkyrie::n_adof);
}

void ValkyrieControlArchitecture::_InitializeTasks(){
    // CoM and Pelvis Tasks
    com_task_ = new CoMxyz(robot_);
    ang_momentum_task_ = new AngularMomentumTask(robot_, ValkyrieAux::servo_rate);
    pelvis_ori_task_ = new BasicTask(robot_, BasicTaskType::LINKORI, 3, ValkyrieBodyNode::pelvis);

    // Set Upper Body Joint Tasks
    upper_body_joint_indices_.clear();
    for(int i = ValkyrieDoF::torsoYaw; i < (ValkyrieDoF::rightForearmYaw + 1); i++){
        upper_body_joint_indices_.push_back(i);
    }
    upper_body_task_ = new SelectedJointTasks(robot_, upper_body_joint_indices_);

    // Set Foot Motion Tasks
    rfoot_center_pos_task_ = new BasicTask(robot_, BasicTaskType::LINKXYZ, 3, ValkyrieBodyNode::rightCOP_Frame);
    lfoot_center_pos_task_ = new BasicTask(robot_, BasicTaskType::LINKXYZ, 3, ValkyrieBodyNode::leftCOP_Frame);
    rfoot_center_ori_task_ = new BasicTask(robot_, BasicTaskType::LINKORI, 3, ValkyrieBodyNode::rightCOP_Frame);
    lfoot_center_ori_task_ = new BasicTask(robot_, BasicTaskType::LINKORI, 3, ValkyrieBodyNode::leftCOP_Frame);
}

void ValkyrieControlArchitecture::_InitializeContacts(){
    rfoot_contact_ = new SurfaceContactSpec(
        robot_, ValkyrieBodyNode::rightCOP_Frame, 0.135, 0.08, 0.7);
    lfoot_contact_ = new SurfaceContactSpec(
        robot_, ValkyrieBodyNode::leftCOP_Frame, 0.135, 0.08, 0.7);
    dim_contact_ = rfoot_contact_->getDim() + lfoot_contact_->getDim(); 
    rfoot_max_z_ = 1500;
    lfoot_max_z_ = 1500;
}

void ValkyrieControlArchitecture::_DeleteController(){
    delete ihwbc_;
    delete ihwbc_joint_integrator_;
}
void ValkyrieControlArchitecture::_DeleteTasks(){
    delete com_task_;
    delete ang_momentum_task_;
    delete pelvis_ori_task_;
    delete upper_body_task_;
    delete rfoot_center_pos_task_;
    delete lfoot_center_pos_task_;
    delete rfoot_center_ori_task_;
    delete lfoot_center_ori_task_;
}
void ValkyrieControlArchitecture::_DeleteContacts(){
    delete rfoot_contact_;
    delete lfoot_contact_;
}