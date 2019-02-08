#include "CartPoleWorldNode.hpp"
#include "PnC/CartPolePnC/CartPoleInterface.hpp"
#include "Utils/IO/DataManager.hpp"
#include "Utils/IO/IOUtilities.hpp"
#include "Configuration.h"

CartPoleWorldNode::CartPoleWorldNode(const dart::simulation::WorldPtr & world_, osgShadow::MinimalShadowMap * msm) : dart::gui::osg::WorldNode(world_, msm)
{
    interface_ = new CartPoleInterface();
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
}
