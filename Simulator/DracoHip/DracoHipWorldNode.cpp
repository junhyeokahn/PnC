#include "DracoHipWorldNode.hpp"
#include "DracoHipPnC/DracoHipInterface.hpp"
#include "DataManager.hpp"
#include "Utilities.hpp"
#include "ParamHandler.hpp"
#include "Configuration.h"

DracoHipWorldNode::DracoHipWorldNode(const dart::simulation::WorldPtr & world_,
                                     osgShadow::MinimalShadowMap * msm) :
    dart::gui::osg::WorldNode(world_, msm) {

    mInterface = new DracoHipInterface();
    mSensorData = new DracoHipSensorData();
    mCommand = new DracoHipCommand();

    mSkel = world_->getSkeleton("DracoFixed");
    mDof = mSkel->getNumDofs();
    mTorqueCommand = Eigen::VectorXd::Zero(mDof);

    ParamHandler handler(THIS_COM"Config/Draco/SIMULATION.yaml");
    handler.getBoolean("IsVisualizeTrajectory", mIsVisualizeTrajectory);
    if (mIsVisualizeTrajectory) {
        std::cout << "[Trajectory Visualization]" << std::endl;
        std::string tmp_string;
        handler.getString("TrajectoryFile", tmp_string);
        mTrajectoryFile =
            THIS_COM"PnC/DracoPnC/OfflineTrajectoryGeneration/TrajectoriesBin/";
        mTrajectoryFile += tmp_string;
        myUtils::readFile(mTrajectoryFile, mFile);
        mLine = new std::string[128];
        mPos = Eigen::VectorXd::Zero(mDof);
        mVel = Eigen::VectorXd::Zero(mDof);
    }
}

DracoHipWorldNode::~DracoHipWorldNode() {
    delete mLine;
    delete mInterface;
    delete mSensorData;
    delete mCommand;
}

void DracoHipWorldNode::customPreStep() {
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
        mInterface->getCommand(mSensorData, mCommand);
        mTorqueCommand = mCommand->jtrq;
        mTorqueCommand.setZero();

        // Gain Testing
        //static Eigen::VectorXd qdes = (mSkel->getPositions()).tail(10);
        //Eigen::VectorXd q = (mSkel->getPositions()).tail(10);
        //Eigen::VectorXd qdot = (mSkel->getVelocities()).tail(10);
        //double v1(500); double v2(500);
        //double v3(10); double v4(2);
        //double kp[10] = {v1, v1, v1, v1, v2,
                         //v1, v1, v1, v1, v2};
        //double kd[10] = {v3, v3, v3, v3, v4,
                         //v3, v3, v3, v3, v4};
        //for (int i = 0; i < 10; ++i) {
            //mTorqueCommand[i+6] = kp[i]*(qdes[i] - q[i]) -
                //kd[i] * qdot[i];
        //}
        //myUtils::pretty_print(q, std::cout, "q");
        //myUtils::pretty_print(qdes, std::cout, "qdes");
    }

    mSkel->setForces(mTorqueCommand);
}
