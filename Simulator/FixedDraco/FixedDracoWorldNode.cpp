#include "FixedDracoWorldNode.hpp"
#include "FixedDracoPnC/FixedDracoInterface.hpp"
#include "DataManager.hpp"
#include "Utilities.hpp"
#include "ParamHandler.hpp"
#include "Configuration.h"

FixedDracoWorldNode::FixedDracoWorldNode(const dart::simulation::WorldPtr & world_,
                                         osgShadow::MinimalShadowMap * msm) :
    dart::gui::osg::WorldNode(world_, msm) {

    mInterface = new FixedDracoInterface();
    mSensorData = new FixedDracoSensorData();
    mCommand = new FixedDracoCommand();

    mSkel = world_->getSkeleton("FixedDraco");
    mDof = mSkel->getNumDofs();
    mTorqueCommand = Eigen::VectorXd::Zero(mDof);
}

FixedDracoWorldNode::~FixedDracoWorldNode() {
    delete mInterface;
    delete mCommand;
    delete mSensorData;
}

void FixedDracoWorldNode::customPreStep() {
    mSensorData->q = mSkel->getPositions();
    mSensorData->qdot = mSkel->getVelocities();

    mInterface->getCommand(mSensorData, mCommand);
    mTorqueCommand.tail(mDof - 6) = mCommand->jtrq;

    // Inv Kin Test
    //mSkel->setPositions(mCommand->q);
    //mSkel->setVelocities(mCommand->qdot);
    //mTorqueCommand.setZero();

    //mTorqueCommand.setZero();
    mSkel->setForces(mTorqueCommand);
}
