#include <Configuration.h>
#include <PnC/ValkyriePnC/ValkyrieInterface.hpp>
#include <Simulator/Dart/Valkyrie/ValkyrieWorldNode.hpp>
#include <Utils/IO/DataManager.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>

ValkyrieWorldNode::ValkyrieWorldNode(const dart::simulation::WorldPtr& _world)
    : dart::gui::osg::WorldNode(_world), count_(0), t_(0.0), servo_rate_(0) {
    world_ = _world;
    robot_ = world_->getSkeleton("valkyrie");
    trq_lb_ = robot_->getForceLowerLimits();
    trq_ub_ = robot_->getForceUpperLimits();
    n_dof_ = robot_->getNumDofs();
    ground_ = world_->getSkeleton("ground_skeleton");
    b_plot_mpc_result_ = false;

    trq_cmd_ = Eigen::VectorXd::Zero(n_dof_);

    interface_ = new ValkyrieInterface();
    sensor_data_ = new ValkyrieSensorData();
    command_ = new ValkyrieCommand();

    SetParams_();
}

ValkyrieWorldNode::~ValkyrieWorldNode() {
    delete interface_;
    delete sensor_data_;
    delete command_;
}

void ValkyrieWorldNode::customPreStep() {
    t_ = (double)count_ * servo_rate_;

    sensor_data_->q = robot_->getPositions().tail(n_dof_ - 6);
    sensor_data_->virtual_q = robot_->getPositions().head(6);
    sensor_data_->qdot = robot_->getVelocities().tail(n_dof_ - 6);
    sensor_data_->virtual_qdot = robot_->getVelocities().head(6);

    // Compute local frame wrenches on the sensor  
    GetForceTorqueData_();
    // Use force thresholding to detect contacts
    GetContactSwitchData_(sensor_data_->rfoot_contact,
                          sensor_data_->lfoot_contact);


    // Walking Interface Example
    // static bool b_first_cmd(true);
    // if (t_ > 1. && b_first_cmd) {
    //     std::cout << "[[first command]]" << std::endl;
    //     ((ValkyrieInterface*)interface_)->Walk(0.15, 0.27, 0.27, 0., 5);
    //     b_first_cmd = false;
    // }
    // static bool b_second_cmd(true);
    // if (t_ > 11. && b_second_cmd) {
    //     std::cout << "[[second command]]" << std::endl;
    //     ((ValkyrieInterface*)interface_)->Walk(0.15, 0.27, 0.27, 0.15, 5);
    //     b_second_cmd = false;
    // }
    // static bool b_third_cmd(true);
    // if (t_ > 21. && b_third_cmd) {
    //     std::cout << "[[third command]]" << std::endl;
    //     ((ValkyrieInterface*)interface_)->Walk(0.15, 0.27, 0.27, -0.15, 5);
    //     b_third_cmd = false;
    // }
    // static bool b_fourth_cmd(true);
    // if (t_ > 31. && b_fourth_cmd) {
    //     std::cout << "[[fourth command]]" << std::endl;
    //     ((ValkyrieInterface*)interface_)->Walk(0., 0.27, 0.27, 0.15, 5);
    //     b_fourth_cmd = false;
    // }
    // static bool b_fifth_cmd(true);
    // if (t_ > 41. && b_fifth_cmd) {
    //     std::cout << "[[fifth command]]" << std::endl;
    //     ((ValkyrieInterface*)interface_)->Walk(-0.1, 0.27, 0.27, 0., 5);
    //     b_fifth_cmd = false;
    // }
    // static bool b_sixth_cmd(true);
    // if (t_ > 55. && b_sixth_cmd) {
    //     std::cout << "[[sixth command]]" << std::endl;
    //     ((ValkyrieInterface*)interface_)->Walk(0., 0.27, 0.2, 0., 7);
    //     b_sixth_cmd = false;
    // }
    // static bool b_seventh_cmd(true);
    // if (t_ > 70. && b_seventh_cmd) {
    //     std::cout << "[[seventh command]]" << std::endl;
    //     ((ValkyrieInterface*)interface_)->Walk(0., 0.2, 0.27, 0., 7);
    //     b_seventh_cmd = false;
    // }

    interface_->getCommand(sensor_data_, command_);

    if (b_plot_mpc_result_) {
        if (((ValkyrieInterface*)interface_)->IsTrajectoryUpdated()) {
            PlotMPCResult_();
        }
    }

    trq_cmd_.tail(n_dof_ - 6) = command_->jtrq;
    for (int i = 0; i < n_dof_ - 6; ++i) {
        trq_cmd_[i + 6] += kp_ * (command_->q[i] - sensor_data_->q[i]) +
                           kd_ * (command_->qdot[i] - sensor_data_->qdot[i]);
    }
    trq_cmd_.head(6).setZero();

    robot_->setForces(trq_cmd_);

    count_++;
}

void ValkyrieWorldNode::PlotMPCResult_() {
    world_->removeAllSimpleFrames();
    std::vector<Eigen::VectorXd> com_des_list;
    std::vector<Eigen::Isometry3d> contact_sequence;

    ((ValkyrieInterface*)interface_)->GetCoMTrajectory(com_des_list);
    ((ValkyrieInterface*)interface_)->GetContactSequence(contact_sequence);
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
                dart::dynamics::BoxShape(Eigen::Vector3d(0.23, 0.16, 0.001)));
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

void ValkyrieWorldNode::GetContactSwitchData_(bool& rfoot_contact,
                                              bool& lfoot_contact) {
    // Get Sensor Wrench Data
    Eigen::VectorXd rf_wrench = sensor_data_->rf_wrench;
    Eigen::VectorXd lf_wrench = sensor_data_->lf_wrench;

    // Local Z-Force Threshold
    double force_threshold = 10; // 10 Newtons ~ 1kg. If sensor detects this force, then we are in contact

    if (fabs(rf_wrench[5]) >= force_threshold) {
        rfoot_contact = true;
    } else {
        rfoot_contact = false;
    }

    if (fabs(lf_wrench[5]) >= force_threshold) {
        lfoot_contact = true;
    } else {
        lfoot_contact = false;
    }

    // std::cout << "rwrench = " << fabs(rf_wrench[5]) << std::endl;
    // std::cout << "lwrench = " << fabs(lf_wrench[5]) << std::endl;
    // std::cout << "Rfoot contact = " << rfoot_contact << std::endl;
    // std::cout << "Lfoot contact = " << lfoot_contact << std::endl;

}

void ValkyrieWorldNode::SetParams_() {
    try {
        YAML::Node simulation_cfg =
            YAML::LoadFile(THIS_COM "Config/Valkyrie/SIMULATION.yaml");
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

void ValkyrieWorldNode::GetForceTorqueData_() {
    Eigen::VectorXd rf_wrench = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd lf_wrench = Eigen::VectorXd::Zero(6);

    dart::dynamics::BodyNode* lfoot_bn = robot_->getBodyNode("leftCOP_Frame");
    dart::dynamics::BodyNode* rfoot_bn = robot_->getBodyNode("rightCOP_Frame");
    const dart::collision::CollisionResult& _result =
        world_->getLastCollisionResult();

    Eigen::VectorXd lf_contact_force_sum = Eigen::VectorXd::Zero(3);
    for (const auto& contact : _result.getContacts()) {
        for (const auto& shapeNode :
             lfoot_bn->getShapeNodesWith<dart::dynamics::CollisionAspect>()) {

            // Ensure that we view the force as external.
            double sgn = 1.0;
            if (shapeNode == contact.collisionObject1->getShapeFrame()){
                sgn = 1.0;
            }
            if (shapeNode == contact.collisionObject2->getShapeFrame()){
                sgn = -1.0;
            }
            // Perform Adjoint Map to local frame wrench
            if (shapeNode == contact.collisionObject1->getShapeFrame() ||
                shapeNode == contact.collisionObject2->getShapeFrame()) {
                Eigen::VectorXd w_c = Eigen::VectorXd::Zero(6);
                w_c.tail(3) = (contact.force*sgn);
                lf_contact_force_sum += (contact.force*sgn);
                Eigen::Isometry3d T_wc = Eigen::Isometry3d::Identity();
                T_wc.translation() = contact.point;
                Eigen::Isometry3d T_wa =
                    robot_->getBodyNode("leftCOP_Frame")
                        ->getTransform(dart::dynamics::Frame::World());
                Eigen::Isometry3d T_ca = T_wc.inverse() * T_wa;
                Eigen::MatrixXd AdT_ca = dart::math::getAdTMatrix(T_ca);
                Eigen::VectorXd w_a = Eigen::VectorXd::Zero(6);
                w_a = AdT_ca.transpose() * w_c;
                lf_wrench += w_a;
            }
        }
        for (const auto& shapeNode :
             rfoot_bn->getShapeNodesWith<dart::dynamics::CollisionAspect>()) {
            // Conditional Check to ensure that we view the force as external.
            double sgn = 1.0;
            if (shapeNode == contact.collisionObject1->getShapeFrame()){
                sgn = 1.0;
            }
            if (shapeNode == contact.collisionObject2->getShapeFrame()){
                sgn = -1.0;
            }
            // Perform Adjoint Map to local frame wrench
            if (shapeNode == contact.collisionObject1->getShapeFrame() ||
                shapeNode == contact.collisionObject2->getShapeFrame()) {
                double normal(contact.normal(2));
                Eigen::VectorXd w_c = Eigen::VectorXd::Zero(6);
                w_c.tail(3) = (contact.force*sgn);
                Eigen::Isometry3d T_wc = Eigen::Isometry3d::Identity();
                T_wc.translation() = contact.point;
                Eigen::Isometry3d T_wa =
                    robot_->getBodyNode("rightCOP_Frame")
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

    // myUtils::pretty_print(lf_contact_force_sum, std::cout, "lf_contact_force_sum");
    // myUtils::pretty_print(rf_wrench, std::cout, "sensor true local rf_wrench");
    // myUtils::pretty_print(lf_wrench, std::cout, "sensor true local lf_wrench ");

    sensor_data_->lf_wrench = lf_wrench;
    sensor_data_->rf_wrench = rf_wrench;
}
