#include "FixedDracoWorldNode.hpp"
#include "PnC/FixedDracoPnC/FixedDracoInterface.hpp"
#include "Utils/DataManager.hpp"
#include "Utils/Utilities.hpp"
#include "Utils/ParamHandler.hpp"
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
    mTorqueCommand = mCommand->jtrq;

    // Inv Kin Test
    //mTorqueCommand.setZero();
    //mSkel->setPositions(mCommand->q);
    //mSkel->setVelocities(mCommand->qdot);
    //mSkel->setPositions(init_q);

    //mTorqueCommand.setZero();
    mSkel->setForces(mTorqueCommand);
}
