#include "PendulumWorldNode.hpp"
#include "PendulumPnC/PendulumInterface.hpp"

PendulumWorldNode::PendulumWorldNode(const dart::simulation::WorldPtr & world_,
                                     osgShadow::MinimalShadowMap * msm) :
    dart::gui::osg::WorldNode(world_, msm) {

    mInterface = new PendulumInterface();
    mSensorData = new PendulumSensorData();

    mSkel = world_->getSkeleton("pendulum");
    mDof = mSkel->getNumDofs();
    mTorqueCommand = Eigen::VectorXd::Zero(mDof);
}

PendulumWorldNode::~PendulumWorldNode() {}

void PendulumWorldNode::customPreStep() {
    mSensorData->q = mSkel->getPositions();
    mSensorData->qdot = mSkel->getVelocities();
    Eigen::VectorXd cmd = mInterface->getCommand(mSensorData);
    mTorqueCommand = cmd;
    mTorqueCommand.setZero();

    mSkel->setForces(mTorqueCommand);
}
