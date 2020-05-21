#include <PnC/ValkyriePnC/TaskAndForceContainers/ValkyrieTaskAndForceContainer.hpp>

ValkyrieTaskAndForceContainer::ValkyrieTaskAndForceContainer(RobotSystem* _robot): TaskAndForceContainer(_robot){
	_InitializeTasks();
	_InitializeContacts();
}

ValkyrieTaskAndForceContainer::~ValkyrieTaskAndForceContainer(){
	_DeleteTasks();
	_DeleteContacts();
}

void ValkyrieTaskAndForceContainer::_InitializeTasks(){
    myUtils::pretty_constructor(2, "Valkyrie Task And Force Container");

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
void ValkyrieTaskAndForceContainer::_InitializeContacts(){
    rfoot_contact_ = new SurfaceContactSpec(
        robot_, ValkyrieBodyNode::rightCOP_Frame, 0.135, 0.08, 0.7);
    lfoot_contact_ = new SurfaceContactSpec(
        robot_, ValkyrieBodyNode::leftCOP_Frame, 0.135, 0.08, 0.7);
    dim_contact_ = rfoot_contact_->getDim() + lfoot_contact_->getDim(); 
    rfoot_max_z_ = 1500;
    lfoot_max_z_ = 1500;

    // Set desired reaction forces
    Fd_des_ = Eigen::VectorXd::Zero(dim_contact_);
}

void ValkyrieTaskAndForceContainer::_DeleteTasks(){
    delete com_task_;
    delete ang_momentum_task_;
    delete pelvis_ori_task_;
    delete upper_body_task_;
    delete rfoot_center_pos_task_;
    delete lfoot_center_pos_task_;
    delete rfoot_center_ori_task_;
    delete lfoot_center_ori_task_;
}

void ValkyrieTaskAndForceContainer::_DeleteContacts(){
    delete rfoot_contact_;
    delete lfoot_contact_;
}

// Set Parameters
void ValkyrieTaskAndForceContainer::ParamInitialization(const YAML::Node& node){

}
