#include <Configuration.h>
#include <PnC/ValkyriePnC/ValkyrieInterface.hpp>
#include <Simulator/Dart/Valkyrie/ValkyrieWorldNode.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>

ValkyrieWorldNode::ValkyrieWorldNode(const dart::simulation::WorldPtr& _world)
    : dart::gui::osg::WorldNode(_world), count_(0), t_(0.0), servo_rate_(0) {
    world_ = _world;
    robot_ = world_->getSkeleton("valkyrie");
    n_dof_ = robot_->getNumDofs();
    ground_ = world_->getSkeleton("ground_skeleton");

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

    trq_cmd_.tail(n_dof_ - 6) = command_->jtrq;
    for (int i = 0; i < n_dof_ - 6; ++i) {
        trq_cmd_[i + 6] += kp_ * (command_->q[i] - sensor_data_->q[i]) +
                           kd_ * (command_->qdot[i] - sensor_data_->qdot[i]);
    }
    trq_cmd_.head(6).setZero();

    trq_cmd_.setZero();
    robot_->setForces(trq_cmd_);

    count_++;
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

    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
    }
}

