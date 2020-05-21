#include <PnC/ValkyriePnC/CtrlSet/ValkyrieMainController.hpp>

ValkyrieMainController::ValkyrieMainController(ValkyrieTaskAndForceContainer* _taf_container, RobotSystem* _robot) : Controller(_robot){
    myUtils::pretty_constructor(2, "Valkyrie Main Controller");

    // Initialize Pointer to the Task and Force Container
    taf_container_ = _taf_container;

    // Initialize State Provider
    sp_ = ValkyrieStateProvider::getStateProvider(robot_);

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

ValkyrieMainController::~ValkyrieMainController(){
    delete ihwbc_;
    delete ihwbc_joint_integrator_;
}

void ValkyrieMainController::getCommand(void* _cmd){   
}

void ValkyrieMainController::ctrlInitialization(const YAML::Node& node){    
    // Defaults
    ihwbc_dt_ = ValkyrieAux::servo_rate;
    lambda_qddot_ = 1e-8;  // Generalized Coord Acceleration 
    lambda_Fr_ = 1e-8;     // Reaction Force Regularization
    w_contact_weight_ = 1e-3;  // Contact Weight
}

// Parent Functions not used
void ValkyrieMainController::oneStep(void* _cmd){}
void ValkyrieMainController::firstVisit(){}
void ValkyrieMainController::lastVisit(){}
bool ValkyrieMainController::endOfPhase(){}