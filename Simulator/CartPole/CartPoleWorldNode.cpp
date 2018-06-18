#include "CartPoleWorldNode.hpp"
#include "CartPolePnC/CartPoleInterface.hpp"

CartPoleWorldNode::CartPoleWorldNode(const dart::simulation::WorldPtr & world_,
                                     osgShadow::MinimalShadowMap * msm) :
    dart::gui::osg::WorldNode(world_, msm) {

    mInterface = new CartPoleInterface();
    mSensorData = new CartPoleSensorData();

    mSkel = world_->getSkeleton("cart_pole");
    mDof = mSkel->getNumDofs();
    mTorqueCommand = Eigen::VectorXd::Zero(mDof);
}

CartPoleWorldNode::~CartPoleWorldNode() {}

void CartPoleWorldNode::customPreStep() {
    mSensorData->q = mSkel->getPositions();
    mSensorData->qdot = mSkel->getVelocities();
    Eigen::VectorXd cmd = mInterface->getCommand(mSensorData);
    mTorqueCommand = cmd;

    mSkel->setForces(mTorqueCommand);
}
