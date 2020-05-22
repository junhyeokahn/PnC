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


    // Add all tasks initially. Remove later as needed.
    task_list_.push_back(com_task_);
    task_list_.push_back(ang_momentum_task_);
    task_list_.push_back(pelvis_ori_task_);
    task_list_.push_back(upper_body_task_);

    task_list_.push_back(rfoot_center_pos_task_);
    task_list_.push_back(lfoot_center_pos_task_);
    task_list_.push_back(rfoot_center_ori_task_);
    task_list_.push_back(lfoot_center_ori_task_);
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

    // Add all contacts initially. Remove later as needed.
    contact_list_.push_back(rfoot_contact_);
    contact_list_.push_back(lfoot_contact_);
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
    task_list_.clear();
}

void ValkyrieTaskAndForceContainer::_DeleteContacts(){
    delete rfoot_contact_;
    delete lfoot_contact_;
    contact_list_.clear();
}

// Set Parameters
void ValkyrieTaskAndForceContainer::paramInitialization(const YAML::Node& node){
    // Use defaults
    // Task Gains
    // COM
    kp_com_ = 50*Eigen::VectorXd::Ones(3); 
    kd_com_ = 5.0*Eigen::VectorXd::Ones(3);
    // Ang Momentum
    kp_ang_mom_ = Eigen::VectorXd::Zero(3); 
    kd_ang_mom_ = 50.0*Eigen::VectorXd::Ones(3); //100.0*Eigen::VectorXd::Ones(3);
    // Pelvis
    kp_pelvis_ = 50*Eigen::VectorXd::Ones(3); 
    kd_pelvis_ = 5.0*Eigen::VectorXd::Ones(3);
    // Upper Body Joint
    kp_upper_body_joint_ = 50.0*Eigen::VectorXd::Ones(upper_body_joint_indices_.size()); 
    kd_upper_body_joint_ = 5.0*Eigen::VectorXd::Ones(upper_body_joint_indices_.size());
    // Foot
    kp_foot_ = 50*Eigen::VectorXd::Ones(3); 
    kd_foot_ = 5.0*Eigen::VectorXd::Ones(3);

    // Task Hierachies
    // Set Hierarchy
    w_task_com_ = 5.0;
    w_task_ang_mom_ = 3.0;
    w_task_pelvis_ = 5.0;
    w_task_upper_body_ = 2.0;
    w_task_rfoot_ = 20.0;
    w_task_lfoot_ = 20.0;

    // Set Task Gains
    com_task_->setGain(kp_com_, kd_com_);
    ang_momentum_task_->setGain(kp_ang_mom_, kd_ang_mom_);
    pelvis_ori_task_->setGain(kp_pelvis_, kd_pelvis_);
    upper_body_task_->setGain(kp_upper_body_joint_, kd_upper_body_joint_);
    rfoot_center_pos_task_->setGain(kp_foot_, kd_foot_);
    rfoot_center_ori_task_->setGain(kp_foot_, kd_foot_);
    lfoot_center_pos_task_->setGain(kp_foot_, kd_foot_);
    lfoot_center_ori_task_->setGain(kp_foot_, kd_foot_);

    // Set Task Hierarchies
    com_task_->setHierarchy(w_task_com_);
    ang_momentum_task_->setHierarchy(w_task_ang_mom_);
    pelvis_ori_task_->setHierarchy(w_task_pelvis_);
    upper_body_task_->setHierarchy(w_task_upper_body_);
    rfoot_center_pos_task_->setHierarchy(w_task_rfoot_);
    rfoot_center_ori_task_->setHierarchy(w_task_rfoot_);
    lfoot_center_pos_task_->setHierarchy(w_task_lfoot_);
    lfoot_center_ori_task_->setHierarchy(w_task_lfoot_);    

    // Set Maximum Forces
    ((SurfaceContactSpec*)rfoot_contact_)->setMaxFz(rfoot_max_z_);    
    ((SurfaceContactSpec*)lfoot_contact_)->setMaxFz(lfoot_max_z_);
}
