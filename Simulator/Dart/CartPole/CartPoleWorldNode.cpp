#include <Configuration.h>
#include <Simulator/Dart/CartPole/CartPoleWorldNode.hpp>
#include <PnC/CartPolePnC/CartPoleInterface.hpp>
#include <Utils/IO/DataManager.hpp>
#include <Utils/IO/IOUtilities.hpp>

CartPoleWorldNode::CartPoleWorldNode(const dart::simulation::WorldPtr& world_,
                                     osgShadow::MinimalShadowMap* msm)
    : dart::gui::osg::WorldNode(world_, msm) {
    interface_ = new CartPoleInterface();
    sensor_data_ = new CartPoleSensorData();
    cmd_ = new CartPoleCommand();

    cart_pole_ = world_->getSkeleton("cart_pole");
    torque_cmd_ = Eigen::VectorXd::Zero(2);
}

CartPoleWorldNode::CartPoleWorldNode(const dart::simulation::WorldPtr& world_,
                                     osgShadow::MinimalShadowMap* msm,
                                     int mpi_idx, int env_idx)
    : dart::gui::osg::WorldNode(world_, msm) {
    interface_ = new CartPoleInterface(mpi_idx, env_idx);
    sensor_data_ = new CartPoleSensorData();
    cmd_ = new CartPoleCommand();

    cart_pole_ = world_->getSkeleton("cart_pole");
    torque_cmd_ = Eigen::VectorXd::Zero(2);
}

CartPoleWorldNode::~CartPoleWorldNode() {
    delete interface_;
    delete sensor_data_;
    delete cmd_;
}

void CartPoleWorldNode::customPreStep() {
    sensor_data_->q = cart_pole_->getPositions();
    sensor_data_->qdot = cart_pole_->getVelocities();

    interface_->getCommand(sensor_data_, cmd_);
    torque_cmd_[0] = cmd_->jtrq;

    cart_pole_->setForces(torque_cmd_);
    if (cmd_->done) {
        ResetSim_();
    }
}

void CartPoleWorldNode::ResetSim_() {
    YAML::Node simulation_cfg =
        YAML::LoadFile(THIS_COM "Config/CartPole/SIMULATION.yaml");
    Eigen::VectorXd init_state_lower_bound, init_state_upper_bound;
    myUtils::readParameter(simulation_cfg, "init_state_lower_bound",
                           init_state_lower_bound);
    myUtils::readParameter(simulation_cfg, "init_state_upper_bound",
                           init_state_upper_bound);

    Eigen::VectorXd q = cart_pole_->getPositions();
    Eigen::VectorXd qdot = cart_pole_->getVelocities();

    std::random_device
        rd;  // Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd());  // Standard mersenne_twister_engine seeded with
                             // rd()

    for (int i = 0; i < 2; ++i) {
        std::uniform_real_distribution<> dis(init_state_lower_bound[i],
                                             init_state_upper_bound[i]);
        q[i] = dis(gen);
    }
    for (int i = 0; i < 2; ++i) {
        std::uniform_real_distribution<> dis(init_state_lower_bound[i + 2],
                                             init_state_upper_bound[i + 2]);
        qdot[i] = dis(gen);
    }
    cart_pole_->setPositions(q);
    cart_pole_->setVelocities(qdot);
    // myUtils::pretty_print(q, std::cout, "q");
    // myUtils::pretty_print(qdot, std::cout, "qdot");
}
