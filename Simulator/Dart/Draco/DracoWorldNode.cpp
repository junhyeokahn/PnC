#include <Configuration.h>
#include <PnC/DracoPnC/DracoInterface.hpp>
#include <Simulator/Dart/Draco/DracoWorldNode.hpp>
#include <Utils/IO/DataManager.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>

DracoWorldNode::DracoWorldNode(const dart::simulation::WorldPtr& _world)
    : dart::gui::osg::WorldNode(_world), count_(0), t_(0.0), servo_rate_(0) {
    world_ = _world;
    robot_ = world_->getSkeleton("Draco");
    trq_lb_ = robot_->getForceLowerLimits();
    trq_ub_ = robot_->getForceUpperLimits();
    n_dof_ = robot_->getNumDofs();
    ground_ = world_->getSkeleton("ground_skeleton");
    b_plot_mpc_result_ = false;

    trq_cmd_ = Eigen::VectorXd::Zero(n_dof_);
    kp_ = Eigen::VectorXd::Zero(n_dof_-6);
    kd_ = Eigen::VectorXd::Zero(n_dof_-6);

    Interface_ = new DracoInterface();
    SensorData_ = new DracoSensorData(); 
    Command_ = new DracoCommand();

    SetParams_();
}

DracoWorldNode::~DracoWorldNode() {
    delete Interface_;
    delete SensorData_;
    delete Command_;
}

void DracoWorldNode::customPreStep() {
    t_ = (double)count_ * servo_rate_;

    SensorData_->q = robot_->getPositions().tail(n_dof_ - 6);
    SensorData_->virtual_q = robot_->getPositions().head(6);
    SensorData_->qdot = robot_->getVelocities().tail(n_dof_ - 6);
    SensorData_->virtual_qdot = robot_->getVelocities().head(6);

    GetContactSwitchData_(SensorData_->rfoot_contact,
                          SensorData_->lfoot_contact);
    GetForceTorqueData_();

    // =========================================================================
    // WalkToRelativePositionAndOrientation Method Example
    // =========================================================================

    static bool b_first_cmd(true);
    if (((DracoInterface*)Interface_)->IsReadyForNextCommand() && b_first_cmd) {
        ((DracoInterface*)Interface_)->WalkInY(-0.9);
        b_first_cmd = false;
    }

    static bool b_seventh_cmd(true);
    if (((DracoInterface*)Interface_)->IsReadyForNextCommand() && b_seventh_cmd) {
        ((DracoInterface*)Interface_)->WalkInX(2.1);
        b_seventh_cmd = false;
    }

    static bool b_second_cmd(true);
    if (((DracoInterface*)Interface_)->IsReadyForNextCommand() && b_second_cmd) {
        //((DracoInterface*)Interface_)->Turn(M_PI/2.0);
        ((DracoInterface*)Interface_)->Turn(1.85);
        b_second_cmd = false;
    }

    static bool b_sixth_cmd(true);
    if (((DracoInterface*)Interface_)->IsReadyForNextCommand() && b_sixth_cmd) {
        ((DracoInterface*)Interface_)->WalkInX(0.4);
        b_sixth_cmd = false;
    }

    static bool b_third_cmd(true);
    if (((DracoInterface*)Interface_)->IsReadyForNextCommand() && b_third_cmd) {
        ((DracoInterface*)Interface_)->WalkInY(-1.0);
        b_third_cmd = false;
    }

    static bool b_fourth_cmd(true);
    if (((DracoInterface*)Interface_)->IsReadyForNextCommand() && b_fourth_cmd) {
        //((DracoInterface*)Interface_)->WalkInX(1.8);
        ((DracoInterface*)Interface_)->WalkInX(0.5);
        b_fourth_cmd = false;
    }

    // =========================================================================
    // Walk Method Example
    // =========================================================================
    //static bool b_first_cmd(true);
    //if (((DracoInterface*)Interface_)->IsReadyForNextCommand() && b_first_cmd) {
        //std::cout << "[[first command]]" << std::endl;
        //((DracoInterface*)Interface_)->Walk(0.05, 0.33, 0.33, 0., 25);
        //b_first_cmd = false;
    //}
    //static bool b_second_cmd(true);
    //if (((DracoInterface*)Interface_)->IsReadyForNextCommand() && b_second_cmd) {
        //std::cout << "[[second command]]" << std::endl;
        //((DracoInterface*)Interface_)->Walk(0.05, 0.33, 0.33, 0.1, 5);
        //b_second_cmd = false;
    //}
    //static bool b_third_cmd(true);
    //if (((DracoInterface*)Interface_)->IsReadyForNextCommand() && b_third_cmd) {
        //std::cout << "[[third command]]" << std::endl;
        //((DracoInterface*)Interface_)->Walk(0.05, 0.33, 0.33, -0.1, 5);
        //b_third_cmd = false;
    //}
    //static bool b_fourth_cmd(true);
    //if (((DracoInterface*)Interface_)->IsReadyForNextCommand() && b_fourth_cmd) {
        //std::cout << "[[fourth command]]" << std::endl;
        //((DracoInterface*)Interface_)->Walk(0., 0.33, 0.33, 0.1, 5);
        //b_fourth_cmd = false;
    //}
    //static bool b_fifth_cmd(true);
    //if (((DracoInterface*)Interface_)->IsReadyForNextCommand() && b_fifth_cmd) {
        //std::cout << "[[fifth command]]" << std::endl;
        //((DracoInterface*)Interface_)->Walk(-0.05, 0.33, 0.33, 0., 5);
        //b_fifth_cmd = false;
    //}
    //static bool b_sixth_cmd(true);
    //if (((DracoInterface*)Interface_)->IsReadyForNextCommand() && b_sixth_cmd) {
        //std::cout << "[[sixth command]]" << std::endl;
        //((DracoInterface*)Interface_)->Walk(0., 0.33, 0.27, 0., 7);
        //b_sixth_cmd = false;
    //}
    //static bool b_seventh_cmd(true);
    //if (((DracoInterface*)Interface_)->IsReadyForNextCommand() && b_seventh_cmd) {
        //std::cout << "[[seventh command]]" << std::endl;
        //((DracoInterface*)Interface_)->Walk(0., 0.3, 0.33, 0., 7);
        //b_seventh_cmd = false;
    //}

    Interface_->getCommand(SensorData_, Command_);

    //std::cout << "------------------------" << std::endl;
    //std::cout << "t :" << t_ << std::endl;
    //std::cout << "q :" << std::endl;
    //std::cout << (SensorData_->q.head(6)) << std::endl;
    //std::cout << "jtrq :"  << std::endl;
    //std::cout << (Command_->jtrq.tail(6))  << std::endl;

    if (b_plot_mpc_result_) {
        if (((DracoInterface*)Interface_)->IsTrajectoryUpdated()) {
            PlotMPCResult_();
        }
    }

    trq_cmd_.tail(n_dof_ - 6) = Command_->jtrq;
    for (int i = 0; i < n_dof_ - 6; ++i) {
        trq_cmd_[i + 6] += kp_[i] * (Command_->q[i] - SensorData_->q[i]) +
                           kd_[i] * (Command_->qdot[i] - SensorData_->qdot[i]);
    }
    trq_cmd_.head(6).setZero();

    robot_->setForces(trq_cmd_);

    count_++;
}

void DracoWorldNode::PlotMPCResult_() {
    world_->removeAllSimpleFrames();
    std::vector<Eigen::VectorXd> com_des_list;
    std::vector<Eigen::Isometry3d> contact_sequence;

    ((DracoInterface*)Interface_)->GetCoMTrajectory(com_des_list);
    ((DracoInterface*)Interface_)->GetContactSequence(contact_sequence);
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

void DracoWorldNode::GetContactSwitchData_(bool& rfoot_contact,
                                              bool& lfoot_contact) {
    Eigen::VectorXd rf = robot_->getBodyNode("rFootCenter")
                             ->getWorldTransform()
                             .translation();
    Eigen::VectorXd lf =
        robot_->getBodyNode("lFootCenter")->getWorldTransform().translation();

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

void DracoWorldNode::SetParams_() {
    try {
        YAML::Node simulation_cfg =
            YAML::LoadFile(THIS_COM "Config/Draco/SIMULATION.yaml");
        myUtils::readParameter(simulation_cfg, "servo_rate", servo_rate_);
        myUtils::readParameter(simulation_cfg["control_configuration"], "kp",
                               kp_);
        myUtils::readParameter(simulation_cfg["control_configuration"], "kd",
                               kd_);
        myUtils::readParameter(simulation_cfg, "plot_mpc_result",
                               b_plot_mpc_result_);

    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
    }
}

void DracoWorldNode::GetForceTorqueData_() {
    Eigen::VectorXd rf_wrench = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd lf_wrench = Eigen::VectorXd::Zero(6);

    dart::dynamics::BodyNode* lfoot_bn = robot_->getBodyNode("lAnkle");
    dart::dynamics::BodyNode* rfoot_bn = robot_->getBodyNode("rAnkle");
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
                    robot_->getBodyNode("lFootCenter")
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
                    robot_->getBodyNode("rFootCenter")
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

    SensorData_->lf_wrench = lf_wrench;
    SensorData_->rf_wrench = rf_wrench;
}
