#include "CartPoleWorldNode.hpp"
#include "CartPolePnC/CartPoleInterface.hpp"
#include "DataManager.hpp"

CartPoleWorldNode::CartPoleWorldNode(const dart::simulation::WorldPtr & world_,
                                     osgShadow::MinimalShadowMap * msm) :
    dart::gui::osg::WorldNode(world_, msm) {

    mInterface = new CartPoleInterface();
    mSensorData = new CartPoleSensorData();

    mSkel = world_->getSkeleton("cart_pole");
    mDof = mSkel->getNumDofs();
    mTorqueCommand = Eigen::VectorXd::Zero(mDof);

    DataManager* dataManager = DataManager::GetDataManager();
    mEffAct = 0.0;
    dataManager->RegisterData(&mEffAct, DOUBLE, "JEffAct");
}

CartPoleWorldNode::~CartPoleWorldNode() {}

void CartPoleWorldNode::customPreStep() {
    mSensorData->q = mSkel->getPositions();
    mSensorData->qdot = mSkel->getVelocities();
    Eigen::VectorXd cmd = mInterface->getCommand(mSensorData);
    mTorqueCommand[0] = cmd[0];

    mSkel->setForces(mTorqueCommand);
    mEffAct = mSkel->getForces()[0];
}
