#include <Configuration.h>
#include <Utils/IO/DataManager.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>
#include <Simulator/Dart/Scorpio/ScorpioWorldNode.hpp>
#include <PnC/DracoPnC/DracoInterface.hpp>
#include <PnC/ScorpioPnC/ScorpioInterface.hpp>
#include <PnC/Scorpio2PnC/ScorpioInterface.hpp>


ScorpioWorldNode::ScorpioWorldNode(const dart::simulation::WorldPtr& _world, EnvInterface *draco, EnvInterface *arm1, EnvInterface *arm2)
    : dart::gui::osg::WorldNode(_world), count_(0),  t_(0.0), servo_rate_(0.001){
    world_ = _world;
    draco_ = world_->getSkeleton("Draco");
    scorpio_ = world_->getSkeleton("Scorpio_Kin");
    first_scorpio_ini_pos = scorpio_->getBodyNode("end_effector")->getTransform().translation();
    scorpio2_ = world_->getSkeleton("Scorpio_Kin2");
    second_scorpio_ini_pos = scorpio2_->getBodyNode("end_effector")->getTransform().translation();
    mGround_ = world_->getSkeleton("ground_skeleton");
    mbox_ = world_->getSkeleton("box");
    des_box_pos = Eigen::VectorXd::Zero(3);
    des_box_pos_6d = Eigen::VectorXd::Zero(6);
    box_ini_pos = mbox_->getBodyNode("baseLink")->getTransform().translation();
    box_fin_pos = Eigen::VectorXd::Zero(3);
    n_dof_scorpio_ = scorpio_->getNumDofs();
    p_dof_scorpio_ = 4;
    a_dof_scorpio_ = n_dof_scorpio_ - p_dof_scorpio_;

    draco_first_is_done=false;
    draco_second_is_done=false;
    first_scorpio_done = false;
    second_scorpio_done = false;

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

    active1__ = scorpio2_->getJoint("joint1");
    active2__ = scorpio2_->getJoint("joint2");
    active3__ = scorpio2_->getJoint("joint5");
    active4__ = scorpio2_->getJoint("joint6");
    active5__ = scorpio2_->getJoint("joint9");
    active6__ = scorpio2_->getJoint("joint10");
    active7__ = scorpio2_->getJoint("joint11");

    scorpio_interface_ = arm1;
    scorpio_sensordata_ = new ScorpioSensorData();
    scorpio_cmd_ = new ScorpioCommand();

    scorpio_interface2_ = arm2;
    scorpio_sensordata2_ = new Scorpio2SensorData();
    scorpio_cmd2_ = new Scorpio2Command();

    draco_interface_= draco;
    draco_sensordata_= new DracoSensorData(); 
    draco_cmd_ = new DracoCommand();

    //scorpio_trq_cmd_ = Eigen::VectorXd::Zero(a_dof_scorpio_);
    //scorpio_trq_cmd2_ = Eigen::VectorXd::Zero(a_dof_scorpio_);


    draco_n_dof_ = draco_->getNumDofs();
    draco_trq_cmd_ = Eigen::VectorXd::Zero(draco_n_dof_);
    draco_kp_ = Eigen::VectorXd::Zero(draco_n_dof_-6);
    draco_kd_ = Eigen::VectorXd::Zero(draco_n_dof_-6);

    b_plot_mpc_result_ = false;

    box_ph = BoxPH::table;

    SetParams_();
}


ScorpioWorldNode::~ScorpioWorldNode() {
    delete scorpio_interface_;
    delete scorpio_sensordata_;
    delete scorpio_cmd_;

    delete scorpio_interface2_;
    delete scorpio_sensordata2_;
    delete scorpio_cmd2_;

    delete draco_interface_;
    delete draco_sensordata_;
    delete draco_cmd_;
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


void ScorpioWorldNode::SetActiveForce2(const Eigen::VectorXd & des_force){
    active1__->setCommand(0,des_force[0]);
    active2__->setCommand(0,des_force[1]);
    active3__->setCommand(0,des_force[2]);
    active4__->setCommand(0,des_force[3]);
    active5__->setCommand(0,des_force[4]);
    active6__->setCommand(0,des_force[5]);
    active7__->setCommand(0,des_force[6]);
}

void ScorpioWorldNode::customPreStep() {
    t_ = (double)count_ * servo_rate_;

    draco_sensordata_->q = draco_->getPositions().tail(draco_n_dof_ - 6);
    draco_sensordata_->virtual_q = draco_->getPositions().head(6);
    draco_sensordata_->qdot = draco_->getVelocities().tail(draco_n_dof_ - 6);
    draco_sensordata_->virtual_qdot = draco_->getVelocities().head(6);

    scorpio_sensordata_->q = scorpio_->getPositions();
    scorpio_sensordata_->qdot = scorpio_->getVelocities();

    scorpio_sensordata2_->q = scorpio2_->getPositions();
    scorpio_sensordata2_->qdot = scorpio2_->getVelocities();

    GetContactSwitchData_(draco_sensordata_->rfoot_contact,
                          draco_sensordata_->lfoot_contact);

    // =====================================
    // Draco is Walking to the first scorpio
    // =====================================

    static bool b_draco_first_cmd(true);
    if (((DracoInterface*)draco_interface_)->IsReadyForNextCommand() && b_draco_first_cmd) {
        ((DracoInterface*)draco_interface_)->WalkInY(-0.9);
        b_draco_first_cmd = false;
    }

    static bool b_draco_second_cmd(true);
    if (((DracoInterface*)draco_interface_)->IsReadyForNextCommand() && b_draco_second_cmd) {
        ((DracoInterface*)draco_interface_)->WalkInX(2.1);
        b_draco_second_cmd = false;
    }

    static bool b_draco_third_cmd(true);
    if (((DracoInterface*)draco_interface_)->IsReadyForNextCommand() && b_draco_third_cmd) {
        //((DracoInterface*)draco_interface_)->Turn(M_PI/2.0);
        ((DracoInterface*)draco_interface_)->Turn(1.85);
        b_draco_third_cmd = false;
    }

    static bool b_draco_fourth_cmd(true);
    if (((DracoInterface*)draco_interface_)->IsReadyForNextCommand() && b_draco_fourth_cmd) {
        ((DracoInterface*)draco_interface_)->WalkInX(0.4);
        b_draco_fourth_cmd = false;
    }

    if (!b_draco_first_cmd && !b_draco_second_cmd && !b_draco_third_cmd && !b_draco_fourth_cmd) {
        draco_first_is_done = true;
    }
    //draco_first_is_done = true;


    // ================================================
    // First Scorpio grasp the box and hand it to Draco
    // ================================================

    static bool b_move_cmd(true);
    if (draco_first_is_done && ((ScorpioInterface*)scorpio_interface_)->IsReadyToMove() && b_move_cmd) {
        //std::cout << "First Moving Command Received" << std::endl;
        ((ScorpioInterface*)scorpio_interface_)->MoveEndEffectorTo(-0.3, -0.4, -0.01);
        b_move_cmd = false;
    }

    static bool b_grasp_cmd(true);
    if (draco_first_is_done && ((ScorpioInterface*)scorpio_interface_)->IsReadyToGrasp() && b_grasp_cmd) {
        //std::cout << "First Grasping Command Received" << std::endl;
        ((ScorpioInterface*)scorpio_interface_)->Grasp();
        b_grasp_cmd = false;
    }

    static bool b_move_while_hold_cmd(true);
    if (draco_first_is_done && ((ScorpioInterface*)scorpio_interface_)->IsReadyToMove() && b_move_while_hold_cmd) {
        box_ph = BoxPH::scorpio;
        //std::cout << "First Moving While Holding Command Received" << std::endl;
        //print_position();
        Eigen::VectorXd rel_pos =  draco_->getBodyNode("IMU")->getTransform().translation() - scorpio_->getBodyNode("end_effector")->getTransform().translation() ;
        //std::cout << "rel pos : " << rel_pos << std::endl;
        ((ScorpioInterface*)scorpio_interface_)->MoveEndEffectorTo(rel_pos[0], rel_pos[1], 0.06);
        b_move_while_hold_cmd = false;
    }

    static bool b_release_cmd(true);
    if (draco_first_is_done && ((ScorpioInterface*)scorpio_interface_)->IsReadyToMove() && b_release_cmd) {
        std::cout << "First Release Command Received" << std::endl;
        b_release_cmd = false;
    }

    static bool b_move_back_to_home(true);
    if (draco_first_is_done && ((ScorpioInterface*)scorpio_interface_)->IsReadyToMove() && b_move_back_to_home) {
        box_ph = BoxPH::draco;
        Eigen::VectorXd rel_pos =  first_scorpio_ini_pos - scorpio_->getBodyNode("end_effector")->getTransform().translation() ;
        //std::cout << "rel pos : " << rel_pos << std::endl;
        ((ScorpioInterface*)scorpio_interface_)->MoveEndEffectorTo(rel_pos[0], rel_pos[1], rel_pos[2]);
        b_move_back_to_home = false;
    }

    if (!b_move_cmd && !b_grasp_cmd && !b_move_while_hold_cmd &&!b_release_cmd && !b_move_back_to_home) {
        first_scorpio_done = true;
    }

    // ==============================================
    // Draco Walks to the second Scorpio with the box
    // ==============================================
    static bool b_draco_fifth_cmd(true);
    if (first_scorpio_done && ((DracoInterface*)draco_interface_)->IsReadyForNextCommand() && b_draco_fifth_cmd) {
        ((DracoInterface*)draco_interface_)->WalkInY(-1.0);
        b_draco_fifth_cmd = false;
    }

    static bool b_draco_sixth_cmd(true);
    if (first_scorpio_done && ((DracoInterface*)draco_interface_)->IsReadyForNextCommand() && b_draco_sixth_cmd) {
        ((DracoInterface*)draco_interface_)->WalkInX(1.6);
        b_draco_sixth_cmd = false;
    }

    if (!b_draco_fifth_cmd && !b_draco_sixth_cmd && ((DracoInterface*)draco_interface_)->IsReadyForNextCommand()) {
        draco_second_is_done=true;
    }

    // ======================================================================
    // Second Scorpio Pick up the box on top of Draco and put it on the table
    // ======================================================================

    static bool b_move_second_cmd(true);
    if (draco_second_is_done && ((Scorpio2Interface*)scorpio_interface2_)->IsReadyToMove() && b_move_second_cmd && ((DracoInterface*)draco_interface_)->IsReadyForNextCommand()) {
        Eigen::VectorXd rel_pos =  mbox_->getBodyNode("baseLink")->getTransform().translation() - scorpio2_->getBodyNode("end_effector")->getTransform().translation() ;
        ((Scorpio2Interface*)scorpio_interface2_)->MoveEndEffectorTo(rel_pos[0], rel_pos[1], rel_pos[2]+0.05);
        b_move_second_cmd = false;
    }

    static bool b_grasp_second_cmd(true);
    if (draco_second_is_done && ((Scorpio2Interface*)scorpio_interface2_)->IsReadyToGrasp() && b_grasp_second_cmd) {
        ((Scorpio2Interface*)scorpio_interface2_)->Grasp();
        b_grasp_second_cmd = false;
    }

    static bool b_move_while_grasp_second_cmd(true);
    if (draco_second_is_done && ((Scorpio2Interface*)scorpio_interface2_)->IsReadyToMove() && b_move_while_grasp_second_cmd) {
        box_ph = BoxPH::scorpio2;
        Eigen::VectorXd rel_pos =  second_scorpio_ini_pos - scorpio2_->getBodyNode("end_effector")->getTransform().translation() ;
        ((Scorpio2Interface*)scorpio_interface2_)->MoveEndEffectorTo(rel_pos[0]-0.2, rel_pos[1]-0.1, rel_pos[2]+0.05);
        b_move_while_grasp_second_cmd = false;
    }


    static bool b_release_second_cmd(true);
    if (draco_second_is_done && ((Scorpio2Interface*)scorpio_interface2_)->IsReadyToMove() && b_release_second_cmd) {
        b_release_second_cmd = false;
    }

    static bool b_move_to_home_second(true);
    if (draco_second_is_done && ((Scorpio2Interface*)scorpio_interface2_)->IsReadyToMove() && b_move_to_home_second) {
        box_ph = BoxPH::table2;
        box_fin_pos = mbox_->getBodyNode("baseLink")->getTransform().translation();
        box_fin_pos[2] = box_ini_pos[2];
        Eigen::VectorXd rel_pos =  second_scorpio_ini_pos - scorpio2_->getBodyNode("end_effector")->getTransform().translation() ;
        ((Scorpio2Interface*)scorpio_interface2_)->MoveEndEffectorTo(rel_pos[0], rel_pos[1], rel_pos[2]);
        b_move_to_home_second = false;
    }

    if (!b_move_second_cmd && !b_grasp_second_cmd && !b_move_while_grasp_second_cmd) {
        second_scorpio_done = true;
    } else {
        // do nothing
    }

    scorpio_interface_->getCommand(scorpio_sensordata_, scorpio_cmd_);
    scorpio_trq_cmd_ = scorpio_cmd_->jtrq;
    SetActiveForce(scorpio_trq_cmd_);

    scorpio_interface2_->getCommand(scorpio_sensordata2_, scorpio_cmd2_);
    scorpio_trq_cmd_ = scorpio_cmd2_->jtrq;
    SetActiveForce2(scorpio_trq_cmd_);


    draco_interface_->getCommand(draco_sensordata_, draco_cmd_);
    draco_trq_cmd_.tail(draco_n_dof_ - 6) = draco_cmd_->jtrq;
    for (int i = 0; i < draco_n_dof_ - 6; ++i) {
        draco_trq_cmd_[i + 6] += draco_kp_[i] * (draco_cmd_->q[i] - draco_sensordata_->q[i]) +
                           draco_kd_[i] * (draco_cmd_->qdot[i] - draco_sensordata_->qdot[i]);
    }
    draco_trq_cmd_.head(6).setZero();
    draco_->setForces(draco_trq_cmd_);

    if (box_ph == BoxPH::table) {
        des_box_pos = box_ini_pos;
    } else if (box_ph == BoxPH::scorpio) {
        des_box_pos = scorpio_->getBodyNode("end_effector")->getTransform().translation();
        des_box_pos[2] -= 0.05;
    } else if (box_ph == BoxPH::draco) {
        des_box_pos = draco_->getBodyNode("IMU")->getTransform().translation();
        des_box_pos[2] += 0.05;
    } else if (box_ph == BoxPH::scorpio2) {
        des_box_pos = scorpio2_->getBodyNode("end_effector")->getTransform().translation();
        des_box_pos[2] -= 0.05;
    } else if (box_ph == BoxPH::table2) {
        des_box_pos = box_fin_pos;
    }
    for (int i = 0; i < 3; ++i) {
        des_box_pos_6d[i] = des_box_pos[i];
    }

    Eigen::VectorXd box_qddot_des = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd box_forces = Eigen::VectorXd::Zero(6);
    for (int i = 0; i < 3; ++i) {
        box_qddot_des[i] = box_kp[i] * (des_box_pos[i] - mbox_->getPositions()[i])
            + box_kd[i] * (- mbox_->getVelocities()[i]);
    }
    box_forces = mbox_->getMassMatrix() * box_qddot_des + mbox_->getCoriolisAndGravityForces();
    mbox_->setForces(box_forces);
    //des_box_pos_6d += 0.00
    //mbox_->setPositions(des_box_pos_6d);

    count_++;

    if (count_ % 100 == 0) {
        //((ScorpioInterface*)scorpio_interface_)->PrintPhase();
        //((Scorpio2Interface*)scorpio_interface2_)->PrintPhase();
    }
}

void ScorpioWorldNode::GetContactSwitchData_(bool& rfoot_contact,
                                             bool& lfoot_contact) {
    Eigen::VectorXd rf = draco_->getBodyNode("rFootCenter")
                             ->getWorldTransform()
                             .translation();
    Eigen::VectorXd lf =
        draco_->getBodyNode("lFootCenter")->getWorldTransform().translation();

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

    myUtils::readParameter(simulation_cfg, "p1", p1_);
    myUtils::readParameter(simulation_cfg, "p2", p2_);
    myUtils::readParameter(simulation_cfg, "x", x_);
    myUtils::readParameter(simulation_cfg, "y", y_);
    myUtils::readParameter(simulation_cfg, "th", th_);
    myUtils::readParameter(simulation_cfg, "x2", x2_);
    myUtils::readParameter(simulation_cfg, "box_kp", box_kp);
    myUtils::readParameter(simulation_cfg, "box_kd", box_kd);
    for (int i = 0; i < 6; ++i) {
        box_kp[i] = box_kd[i] * box_kd[i] / 4.;
    }
    myUtils::readParameter(simulation_cfg, "draco_top_height", draco_top_height_);

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

void ScorpioWorldNode::print_position(){

    Eigen::VectorXd scorpio1__;
    scorpio1__ = scorpio_->getBodyNode("end_effector")->getTransform().translation();
    myUtils::pretty_print(scorpio1__, std::cout, "First Scorpio Endeffection");

    Eigen::VectorXd scorpio2__;
    scorpio2__ = scorpio2_->getBodyNode("end_effector")->getTransform().translation();
    myUtils::pretty_print(scorpio2__, std::cout, "Second Scorpio Endeffection");

    Eigen::VectorXd draco__;
    draco__ = draco_->getBodyNode("IMU")->getTransform().translation();
    myUtils::pretty_print(draco__, std::cout, "Draco IMU");

    //Eigen::VectorXd box__;
    //box__ = draco_->getBodyNode("IMU")->getTransform().translation();
    //myUtils::pretty_print(box__, std::cout, "Box pos");

}
