#include <Configuration.h>
#include <Utils/IO/DataManager.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>
#include <Simulator/Dart/Scorpio/ScorpioWorldNode.hpp>
#include <PnC/DracoPnC/DracoInterface.hpp>
#include <PnC/ScorpioPnC/ScorpioInterface.hpp>


ScorpioWorldNode::ScorpioWorldNode(const dart::simulation::WorldPtr& _world)
    : dart::gui::osg::WorldNode(_world), count_(0),  t_(0.0), servo_rate_(0.001){
    world_ = _world;
    draco_ = world_->getSkeleton("Draco");
    scorpio_ = world_->getSkeleton("Scorpio_Kin");
    mGround_ = world_->getSkeleton("ground_skeleton");
    trq_lb_scorpio_ = scorpio_->getForceLowerLimits();
    trq_ub_scorpio_ = scorpio_->getForceUpperLimits();
    n_dof_scorpio_ = scorpio_->getNumDofs();
    p_dof_scorpio_ = 4;
    a_dof_scorpio_ = n_dof_scorpio_ - p_dof_scorpio_;

    q_init_ = scorpio_->getPositions();

    active_joint_idx_.resize(a_dof_scorpio_);
    active_joint_idx_[0] = scorpio_->getDof("joint1")->getIndexInSkeleton();
    active_joint_idx_[1] = scorpio_->getDof("joint2")->getIndexInSkeleton();
    active_joint_idx_[2] = scorpio_->getDof("joint5")->getIndexInSkeleton();
    active_joint_idx_[3] = scorpio_->getDof("joint6")->getIndexInSkeleton();
    active_joint_idx_[4] = scorpio_->getDof("joint9")->getIndexInSkeleton();
    active_joint_idx_[5] = scorpio_->getDof("joint10")->getIndexInSkeleton();
    active_joint_idx_[6] = scorpio_->getDof("joint11")->getIndexInSkeleton();

    passive_joint_idx_.resize(p_dof_scorpio_);
    passive_joint_idx_[0] = scorpio_->getDof("joint3")->getIndexInSkeleton();
    passive_joint_idx_[1] = scorpio_->getDof("joint4")->getIndexInSkeleton();
    passive_joint_idx_[2] = scorpio_->getDof("joint7")->getIndexInSkeleton();
    passive_joint_idx_[3] = scorpio_->getDof("joint8")->getIndexInSkeleton();

    active1_ = scorpio_->getJoint("joint1");
    active2_ = scorpio_->getJoint("joint2");
    active3_ = scorpio_->getJoint("joint5");
    active4_ = scorpio_->getJoint("joint6");
    active5_ = scorpio_->getJoint("joint9");
    active6_ = scorpio_->getJoint("joint10");
    active7_ = scorpio_->getJoint("joint11");

    scorpio_interface_ = new ScorpioInterface();
    scorpio_sensordata_ = new ScorpioSensorData();
    scorpio_cmd_ = new ScorpioCommand();

    draco_interface_= new DracoInterface();
    draco_sensordata_= new DracoSensorData(); 
    draco_cmd_ = new DracoCommand();

    //scorpio_trq_cmd_ = Eigen::VectorXd::Zero(a_dof_scorpio_);


    draco_n_dof_ = draco_->getNumDofs();
    draco_trq_cmd_ = Eigen::VectorXd::Zero(draco_n_dof_);
    draco_kp_ = Eigen::VectorXd::Zero(draco_n_dof_-6);
    draco_kd_ = Eigen::VectorXd::Zero(draco_n_dof_-6);

    b_plot_mpc_result_ = false;

    SetParams_();
}


ScorpioWorldNode::~ScorpioWorldNode() {
    delete scorpio_interface_;
    delete scorpio_sensordata_;
    delete scorpio_cmd_;
    delete draco_interface_;
    delete draco_sensordata_;
    delete draco_cmd_;
}

void ScorpioWorldNode::GetActiveJointInfo(Eigen::VectorXd & cur_pos, Eigen::VectorXd & cur_vel){
    Eigen::VectorXd j_pos(n_dof_scorpio_);
    Eigen::VectorXd j_vel(n_dof_scorpio_);
    j_pos = scorpio_->getPositions();
    j_vel = scorpio_->getVelocities();

    cur_pos.resize(a_dof_scorpio_);
    cur_pos.setZero();
    cur_vel.resize(a_dof_scorpio_);
    cur_vel.setZero();

    for(int i(0); i< a_dof_scorpio_; ++i){
        cur_pos[i] = j_pos[active_joint_idx_[i]];
        cur_vel[i] = j_vel[active_joint_idx_[i]];
    }
}

void ScorpioWorldNode::SetActivePosition(const Eigen::VectorXd & des_pos){
    for(int i(0); i<a_dof_scorpio_; ++i)
        scorpio_->setPosition(active_joint_idx_[i], des_pos[i]);
}

void ScorpioWorldNode::SetActiveForce(const Eigen::VectorXd & des_force){
 
    active1_->setCommand(0,des_force[0]);
    active2_->setCommand(0,des_force[1]);
    active3_->setCommand(0,des_force[2]);
    active4_->setCommand(0,des_force[3]);
    active5_->setCommand(0,des_force[4]);
    active6_->setCommand(0,des_force[5]);
    active7_->setCommand(0,des_force[6]);
}

void ScorpioWorldNode::UpdateSystem(const Eigen::VectorXd & q_activate){
    Eigen::VectorXd q_full(n_dof_scorpio_);

    q_full[0] = q_activate[0];
    q_full[1] = q_activate[1];
    q_full[2] = -q_activate[1];
    q_full[3] = q_activate[1];
    q_full[4] = q_activate[2];
    q_full[5] = q_activate[3];
    q_full[6] = -q_activate[3];
    q_full[7] = q_activate[3];
    q_full[8] = q_activate[4];
    q_full[9] = q_activate[5];
    q_full[10] = q_activate[6];

    scorpio_->setPositions(q_full);
    scorpio_->computeForwardKinematics(true, false, false);
}

void ScorpioWorldNode::SetActiveVelocity(const Eigen::VectorXd & des_vel){

    active1_->setCommand(0,des_vel[0]);
    active2_->setCommand(0,des_vel[1]);
    active3_->setCommand(0,des_vel[2]);
    active4_->setCommand(0,des_vel[3]);
    active5_->setCommand(0,des_vel[4]);
    active6_->setCommand(0,des_vel[5]);
    active7_->setCommand(0,des_vel[6]);

}

void ScorpioWorldNode::customPreStep() {
    t_ = (double)count_ * servo_rate_;


    // =============
    // Scorpio
    // =============
    scorpio_sensordata_->q = scorpio_->getPositions();
    scorpio_sensordata_->qdot = scorpio_->getVelocities();

    // ====
    // APIs
    // ====
    static bool b_sc_first_cmd(true);
    if (((ScorpioInterface*)scorpio_interface_)->IsReadyToMove() && b_sc_first_cmd) {
        std::cout << "First Command Received" << std::endl;
        ((ScorpioInterface*)scorpio_interface_)->MoveEndEffectorTo(-1.0, 0.1, 0.);
        b_sc_first_cmd = false;
    }

    static bool is_grasping = false;
    //static bool b_sc_second_cmd(true);
    //if (((ScorpioInterface*)scorpio_interface_)->IsReadyToGrasp() && b_sc_second_cmd) {
        //std::cout << "Second Command Received" << std::endl;
        //((ScorpioInterface*)scorpio_interface_)->Grasp();
        //b_sc_second_cmd = false;
        is_grasping = true;
    //}

    scorpio_interface_->getCommand(scorpio_sensordata_, scorpio_cmd_);

    if (is_grasping) {
        // set force to box
    } else {
        // set force to stay at the same position
    }

    static bool b_sc_third_cmd(true);
    if (((ScorpioInterface*)scorpio_interface_)->IsReadyToMove() && b_sc_third_cmd) {
        std::cout << "Third Command Received" << std::endl;
        ((ScorpioInterface*)scorpio_interface_)->MoveEndEffectorTo(-1.0, 0.1, 0.);
        b_sc_third_cmd = false;
    }

    scorpio_trq_cmd_ = scorpio_cmd_->jtrq;
    SetActiveForce(scorpio_trq_cmd_);

    // =============
    // Draco
    // =============
    draco_sensordata_->q = draco_->getPositions().tail(draco_n_dof_ - 6);
    draco_sensordata_->virtual_q = draco_->getPositions().head(6);
    draco_sensordata_->qdot = draco_->getVelocities().tail(draco_n_dof_ - 6);
    draco_sensordata_->virtual_qdot = draco_->getVelocities().head(6);

    GetContactSwitchData_(draco_sensordata_->rfoot_contact,
                          draco_sensordata_->lfoot_contact);

    // ====
    // APIs
    // ====
    static bool b_first_cmd(true);
    if (((DracoInterface*)draco_interface_)->IsReadyForNextCommand() && b_first_cmd) {
        ((DracoInterface*)draco_interface_)->WalkInY(-0.9);
        b_first_cmd = false;
    }

    static bool b_second_cmd(true);
    if (((DracoInterface*)draco_interface_)->IsReadyForNextCommand() && b_second_cmd) {
        ((DracoInterface*)draco_interface_)->WalkInX(2.1);
        b_second_cmd = false;
    }

    static bool b_third_cmd(true);
    if (((DracoInterface*)draco_interface_)->IsReadyForNextCommand() && b_third_cmd) {
        ((DracoInterface*)draco_interface_)->Turn(M_PI/2.0);
        b_third_cmd = false;
    }

    //static bool b_fourth_cmd(true);
    //if (((DracoInterface*)draco_interface_)->IsReadyForNextCommand() && b_fourth_cmd) {
        //((DracoInterface*)draco_interface_)->WalkInX(0.61);
        //b_fourth_cmd = false;
    //}

    draco_interface_->getCommand(draco_sensordata_, draco_cmd_);

    //std::cout << "------------------------" << std::endl;
    //std::cout << "t :" << t_ << std::endl;
    //std::cout << "q :" << std::endl;
    //std::cout << (draco_sensordata_->q.head(6)) << std::endl;
    //std::cout << "jtrq :"  << std::endl;
    //std::cout << (draco_cmd_->jtrq.tail(6))  << std::endl;

    if (b_plot_mpc_result_) {
        if (((DracoInterface*)draco_interface_)->IsTrajectoryUpdated()) {
            PlotMPCResult_();
        }
    }

    draco_trq_cmd_.tail(draco_n_dof_ - 6) = draco_cmd_->jtrq;
    for (int i = 0; i < draco_n_dof_ - 6; ++i) {
        draco_trq_cmd_[i + 6] += draco_kp_[i] * (draco_cmd_->q[i] - draco_sensordata_->q[i]) +
                           draco_kd_[i] * (draco_cmd_->qdot[i] - draco_sensordata_->qdot[i]);
    }
    draco_trq_cmd_.head(6).setZero();

    draco_->setForces(draco_trq_cmd_);

    count_++;
}

void ScorpioWorldNode::GetContactSwitchData_(bool& rfoot_contact,
                                             bool& lfoot_contact) {
    Eigen::VectorXd rf = draco_->getBodyNode("rFootCenter")
                             ->getWorldTransform()
                             .translation();
    Eigen::VectorXd lf =
        draco_->getBodyNode("lFootCenter")->getWorldTransform().translation();

    // myUtils::pretty_print(rf, std::cout, "right_sole");
    // myUtils::pretty_print(lf, std::cout, "left_sole");

    if (fabs(rf[2] < 0.005)) {
        rfoot_contact = true;
        // printf("right contact\n");
    } else {
        rfoot_contact = false;
    }

    if (fabs(lf[2] < 0.005)) {
        lfoot_contact = true;
        // printf("left contact\n");
    } else {
        lfoot_contact = false;
    }
}


void ScorpioWorldNode::SetJointSpaceControlCmd(int ctrl_case){
    if(ctrl_case != 0){
        std::cout<<"[error] Control parameter is not for joint space control in yaml file." <<std::endl;
        exit(0);
    }

    Eigen::VectorXd pos_des(a_dof_scorpio_);
    Eigen::VectorXd pos_cur(a_dof_scorpio_);
    Eigen::VectorXd vel_cur(a_dof_scorpio_);
    Eigen::VectorXd torque_des(a_dof_scorpio_);

    GetActiveJointInfo(pos_cur, vel_cur);
    switch(sim_case_){
        case 0: {
            pos_des[0] = q_init_[active_joint_idx_[0]]+0.;
            pos_des[1] = q_init_[active_joint_idx_[1]]+Amp_[1]/180.0*M_PI*sin((2*M_PI)*Freq_[1]*t_);
            pos_des[2] = q_init_[active_joint_idx_[2]]+0.;
            pos_des[3] = q_init_[active_joint_idx_[3]]+Amp_[3]/180.0*M_PI*sin((2*M_PI)*Freq_[3]*t_);
            pos_des[4] = q_init_[active_joint_idx_[4]]+0.;
            pos_des[5] = q_init_[active_joint_idx_[5]]+0.;
            pos_des[6] = q_init_[active_joint_idx_[6]]+0.;
            break;
        }
        case 1:{
            pos_des[0] = q_init_[active_joint_idx_[0]]+Amp_[0]/180.0*M_PI*sin(Freq_[0]*(2*M_PI)*t_);
            pos_des[1] = q_init_[active_joint_idx_[1]]+0.;
            pos_des[2] = q_init_[active_joint_idx_[2]]-Amp_[2]/180.0*M_PI*sin(Freq_[2]*((2*M_PI)*t_-M_PI/2))-Amp_[2]/180.0*M_PI;
            pos_des[3] = q_init_[active_joint_idx_[3]]+0.;
            pos_des[4] = q_init_[active_joint_idx_[4]]-Amp_[4]/180.0*M_PI*sin(Freq_[4]*((2*M_PI)*t_-M_PI/2))-Amp_[4]/180.0*M_PI;
            pos_des[5] = q_init_[active_joint_idx_[5]]+0.;
            pos_des[6] = q_init_[active_joint_idx_[6]]+0.;
            break;
        }
        case 2:{
            pos_des[0] = q_init_[active_joint_idx_[0]]+Amp_[0]/180.0*M_PI*sin(Freq_[0]*(2*M_PI)*t_);
            pos_des[1] = q_init_[active_joint_idx_[1]]+Amp_[1]/180.0*M_PI*sin((2*M_PI)*Freq_[1]*t_);
            pos_des[2] = q_init_[active_joint_idx_[2]]-Amp_[2]/180.0*M_PI*sin(Freq_[2]*((2*M_PI)*t_-M_PI/2))-Amp_[2]/180.0*M_PI;
            pos_des[3] = q_init_[active_joint_idx_[3]]+Amp_[3]/180.0*M_PI*sin((2*M_PI)*Freq_[3]*t_);
            pos_des[4] = q_init_[active_joint_idx_[4]]-Amp_[4]/180.0*M_PI*sin(Freq_[4]*((2*M_PI)*t_-M_PI/2))-Amp_[4]/180.0*M_PI;
            pos_des[5] = q_init_[active_joint_idx_[5]]+0.;
            pos_des[6] = q_init_[active_joint_idx_[6]]+0.;
            break;
        }
        default:
            pos_des.setZero();
    }

    switch(actuator_type_){
        case 0:{
            Eigen::VectorXd vel_des(a_dof_scorpio_);
            vel_des.setZero();

            vel_des = (pos_des- pos_cur)/servo_rate_;
            SetActiveVelocity(vel_des);
            break;
        }
        case 1:{     
            // SetActiveForce(torque_des);
            torque_des.resize(a_dof_scorpio_);
            torque_des.setZero();

            for( int i(0); i< a_dof_scorpio_; ++i ){
               torque_des[i] = scorpio_kp_[i]*(pos_des[i] - pos_cur[i]) - scorpio_kd_[i]*vel_cur[i];
            } 
            SetActiveForce(torque_des);
            break;
        }
        default:{
            UpdateSystem(pos_des);
            break;
        }
    }

    //myUtils::pretty_print(pos_des, std::cout, "pos_des");
    //myUtils::pretty_print(pos_cur, std::cout, "pos_cur");
   count_++;
}

void ScorpioWorldNode::SetParams_(){
    YAML::Node simulation_cfg = 
         YAML::LoadFile(THIS_COM "Config/Scorpio/SIMULATION.yaml");
    // 0: servo, 1: force
    myUtils::readParameter(simulation_cfg, "actuator_type", actuator_type_);
    YAML::Node control_cfg = 
        YAML::LoadFile(THIS_COM "Config/Scorpio/CONTROL.yaml");
    myUtils::readParameter(control_cfg, "Amp", Amp_ );
    myUtils::readParameter(control_cfg, "Freq", Freq_);
    myUtils::readParameter(control_cfg, "sim_case", sim_case_);
    myUtils::readParameter(control_cfg, "control_type", control_type_);
    myUtils::readParameter(control_cfg, "kp", scorpio_kp_);
    myUtils::readParameter(control_cfg, "kd", scorpio_kd_);

    //myUtils::pretty_print(Amp_, std::cout, "Amp");
    //myUtils::pretty_print(Freq_, std::cout, "Freq");
    //std::cout<<"sim_case: "<<sim_case_ <<std::endl;
    
    //if(actuator_type_ == 0)
        //std::cout<<"actuator_type: servo "<<std::endl;
    //else
        //std::cout<<"actuator_type: force "<<std::endl;

    //if(control_type_ == 0)
        //std::cout<<"control_type: joint space " <<std::endl;
    //else if(control_type_ == 1)
        //std::cout<<"control_type: operational space " <<std::endl;
    //else
        //std::cout<<"control_type: not defined " <<std::endl;

    try {
        YAML::Node simulation_cfg =
            YAML::LoadFile(THIS_COM "Config/Draco/SIMULATION.yaml");
        myUtils::readParameter(simulation_cfg, "servo_rate", servo_rate_);
        myUtils::readParameter(simulation_cfg["control_configuration"], "kp",
                               draco_kp_);
        myUtils::readParameter(simulation_cfg["control_configuration"], "kd",
                               draco_kd_);
        myUtils::readParameter(simulation_cfg, "plot_mpc_result",
                               b_plot_mpc_result_);

    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
    }

}


void ScorpioWorldNode::GetForceTorqueData_() {
    Eigen::VectorXd rf_wrench = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd lf_wrench = Eigen::VectorXd::Zero(6);

    dart::dynamics::BodyNode* lfoot_bn = draco_->getBodyNode("lAnkle");
    dart::dynamics::BodyNode* rfoot_bn = draco_->getBodyNode("rAnkle");
    const dart::collision::CollisionResult& _result =
        world_->getLastCollisionResult();

    for (const auto& contact : _result.getContacts()) {
        for (const auto& shapeNode :
             lfoot_bn->getShapeNodesWith<dart::dynamics::CollisionAspect>()) {
            if (shapeNode == contact.collisionObject1->getShapeFrame() ||
                shapeNode == contact.collisionObject2->getShapeFrame()) {
                double normal(contact.normal(2));
                Eigen::VectorXd w_c = Eigen::VectorXd::Zero(6);
                w_c.tail(3) = contact.force * normal;
                Eigen::Isometry3d T_wc = Eigen::Isometry3d::Identity();
                T_wc.translation() = contact.point;
                Eigen::Isometry3d T_wa =
                    draco_->getBodyNode("lFootCenter")
                        ->getTransform(dart::dynamics::Frame::World());
                Eigen::Isometry3d T_ca = T_wc.inverse() * T_wa;
                Eigen::MatrixXd AdT_ca = dart::math::getAdTMatrix(T_ca);
                Eigen::VectorXd w_a = Eigen::VectorXd::Zero(6);
                w_a = AdT_ca.transpose() * w_c;
                // myUtils::pretty_print(w_a, std::cout, "left");
                lf_wrench += w_a;
            }
        }

        for (const auto& shapeNode :
             rfoot_bn->getShapeNodesWith<dart::dynamics::CollisionAspect>()) {
            if (shapeNode == contact.collisionObject1->getShapeFrame() ||
                shapeNode == contact.collisionObject2->getShapeFrame()) {
                double normal(contact.normal(2));
                Eigen::VectorXd w_c = Eigen::VectorXd::Zero(6);
                w_c.tail(3) = contact.force * normal;
                Eigen::Isometry3d T_wc = Eigen::Isometry3d::Identity();
                T_wc.translation() = contact.point;
                Eigen::Isometry3d T_wa =
                    draco_->getBodyNode("rFootCenter")
                        ->getTransform(dart::dynamics::Frame::World());
                Eigen::Isometry3d T_ca = T_wc.inverse() * T_wa;
                Eigen::MatrixXd AdT_ca = dart::math::getAdTMatrix(T_ca);
                Eigen::VectorXd w_a = Eigen::VectorXd::Zero(6);
                w_a = AdT_ca.transpose() * w_c;
                // myUtils::pretty_print(w_a, std::cout, "right");
                rf_wrench += w_a;
            }
        }
    }

    draco_sensordata_->lf_wrench = lf_wrench;
    draco_sensordata_->rf_wrench = rf_wrench;
}

void ScorpioWorldNode::PlotMPCResult_() {
    world_->removeAllSimpleFrames();
    std::vector<Eigen::VectorXd> com_des_list;
    std::vector<Eigen::Isometry3d> contact_sequence;

    ((DracoInterface*)draco_interface_)->GetCoMTrajectory(com_des_list);
    ((DracoInterface*)draco_interface_)->GetContactSequence(contact_sequence);
    int n_traj = com_des_list.size();
    int n_contact = contact_sequence.size();

    // line segment
    Eigen::Vector4d foot_color = Eigen::Vector4d(1.0, 0.63, 0.0, 1.0);
    Eigen::Vector4d line_color = Eigen::Vector4d(0.9, 0., 0., 1.0);
    Eigen::Vector4d com_color = Eigen::Vector4d(0, 0, 139, 1.0);

    std::vector<dart::dynamics::SimpleFramePtr> line_frame;
    line_frame.clear();

    for (int i = 0; i < n_traj - 1; ++i) {
        Eigen::Vector3d v0, v1;
        v0 << com_des_list[i][0], com_des_list[i][1], com_des_list[i][2];
        v1 << com_des_list[i + 1][0], com_des_list[i + 1][1],
            com_des_list[i + 1][2];
        line_frame.push_back(std::make_shared<dart::dynamics::SimpleFrame>(
            dart::dynamics::Frame::World(), "l" + std::to_string(i)));
        dart::dynamics::LineSegmentShapePtr traj_line =
            std::make_shared<dart::dynamics::LineSegmentShape>(v0, v1, 5.0);
        line_frame[i]->setShape(traj_line);
        line_frame[i]->createVisualAspect();
        line_frame[i]->getVisualAspect()->setColor(line_color);
        world_->addSimpleFrame(line_frame[i]);
    }

    // contact sequence
    std::vector<dart::dynamics::SimpleFramePtr> contact_frame;
    contact_frame.clear();

    for (int i = 0; i < n_contact; ++i) {
        Eigen::Isometry3d tf = contact_sequence[i];
        contact_frame.push_back(std::make_shared<dart::dynamics::SimpleFrame>(
            dart::dynamics::Frame::World(), "c" + std::to_string(i), tf));
        dart::dynamics::BoxShapePtr b_shape =
            std::make_shared<dart::dynamics::BoxShape>(
                dart::dynamics::BoxShape(Eigen::Vector3d(0.17, 0.04, 0.001)));
        contact_frame[i]->setShape(b_shape);
        contact_frame[i]->getVisualAspect(true)->setColor(foot_color);
        world_->addSimpleFrame(contact_frame[i]);
    }

    // com
    // Eigen::Isometry3d com_tf = Eigen::Isometry3d::Identity();
    // com_tf.translation() = robot_->getCOM();
    // dart::dynamics::SimpleFramePtr com_frame =
    // std::make_shared<dart::dynamics::SimpleFrame>(
    // dart::dynamics::Frame::World(), "com", com_tf);
    // std::shared_ptr<dart::dynamics::SphereShape> s_shape =
    // std::make_shared<dart::dynamics::SphereShape>(
    // dart::dynamics::SphereShape(0.05));
    // com_frame->setShape(s_shape);
    // com_frame->getVisualAspect(true)->setColor(com_color);
    // world_->addSimpleFrame(com_frame);
}
