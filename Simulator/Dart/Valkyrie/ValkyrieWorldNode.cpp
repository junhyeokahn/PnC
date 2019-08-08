#include <Configuration.h>
#include <PnC/ValkyriePnC/ValkyrieInterface.hpp>
#include <Simulator/Dart/Valkyrie/ValkyrieWorldNode.hpp>
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

    GetContactSwitchData_(sensor_data_->rfoot_contact,
                          sensor_data_->lfoot_contact);

    interface_->getCommand(sensor_data_, command_);

    if (b_plot_mpc_result_) PlotMPCResult_();

    trq_cmd_.tail(n_dof_ - 6) = command_->jtrq;
    for (int i = 0; i < n_dof_ - 6; ++i) {
        trq_cmd_[i + 6] += kp_ * (command_->q[i] - sensor_data_->q[i]) +
                           kd_ * (command_->qdot[i] - sensor_data_->qdot[i]);
    }
    trq_cmd_.head(6).setZero();

    // trq_cmd_.setZero();
    Eigen::VectorXd clipped_trq =
        myUtils::CropVector(trq_cmd_, trq_lb_, trq_ub_, "final trq");
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
    Eigen::VectorXd rf = robot_->getBodyNode("rightCOP_Frame")
                             ->getWorldTransform()
                             .translation();
    Eigen::VectorXd lf =
        robot_->getBodyNode("leftCOP_Frame")->getWorldTransform().translation();

    // myUtils::pretty_print(rf, std::cout, "right_sole");
    // myUtils::pretty_print(lf, std::cout, "left_sole");

    if (fabs(rf[2] < 0.01)) {
        rfoot_contact = true;
        // printf("right contact\n");
    } else {
        rfoot_contact = false;
    }

    if (fabs(lf[2] < 0.01)) {
        lfoot_contact = true;
        // printf("left contact\n");
    } else {
        lfoot_contact = false;
    }
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

