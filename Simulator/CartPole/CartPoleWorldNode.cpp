#include "CartPoleWorldNode.hpp"
#include "CartPolePnC/CartPoleInterface.hpp"
#include "DataManager.hpp"
#include "Utilities.hpp"
#include "ParamHandler.hpp"
#include "Configuration.h"

CartPoleWorldNode::CartPoleWorldNode(const dart::simulation::WorldPtr & world_,
                                     osgShadow::MinimalShadowMap * msm) :
    dart::gui::osg::WorldNode(world_, msm) {

    mInterface = new CartPoleInterface();
    mSensorData = new CartPoleSensorData();

    mSkel = world_->getSkeleton("cart_pole");
    mDof = mSkel->getNumDofs();
    mTorqueCommand = Eigen::VectorXd::Zero(mDof);

    ParamHandler handler(THIS_COM"Config/CartPole/SIMULATION.yaml");
    handler.getBoolean("IsVisualizeTrajectory", mIsVisualizeTrajectory);
    if (mIsVisualizeTrajectory) {
        std::cout << "[Trajectory Visualization]" << std::endl;
        std::string tmp_string;
        handler.getString("TrajectoryFile", tmp_string);
        mTrajectoryFile =
            THIS_COM"PnC/CartPolePnC/OfflineTrajectoryGeneration/TrajectoriesBin/";
        mTrajectoryFile += tmp_string;
        myUtils::readFile(mTrajectoryFile, mFile);
        mLine = new std::string[128];
        mPos = Eigen::VectorXd::Zero(mDof);
        mVel = Eigen::VectorXd::Zero(mDof);
    }
}

CartPoleWorldNode::~CartPoleWorldNode() {
    delete mLine;
}

void CartPoleWorldNode::customPreStep() {
    mSensorData->q = mSkel->getPositions();
    mSensorData->qdot = mSkel->getVelocities();

    if (mIsVisualizeTrajectory) {
        static int i = 0;
        myUtils::splitString(mLine, mFile[i], "\t");
        for (int j = 0; j < mDof; ++j) {
            mPos[j] = std::stod(mLine[j + 1]);
            mVel[j] = std::stod(mLine[j + 1 + mDof]);
            mTorqueCommand[j] = 100. * (mPos[j] - mSensorData->q[j]) +
                10. * (mVel[j] - mSensorData->qdot[j]);
        }
        ++i;
        if (i == mFile.size())  exit(0);
    } else {
        Eigen::VectorXd cmd = mInterface->getCommand(mSensorData);
        mTorqueCommand[0] = cmd[0];
    }

    mSkel->setForces(mTorqueCommand);
}
