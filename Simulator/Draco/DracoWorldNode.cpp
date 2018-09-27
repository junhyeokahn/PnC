#include "DracoWorldNode.hpp"
#include "PnC/DracoPnC/DracoInterface.hpp"
#include "Utils/DataManager.hpp"
#include "Utils/Utilities.hpp"
#include "Utils/ParamHandler.hpp"
#include "Configuration.h"

DracoWorldNode::DracoWorldNode(const dart::simulation::WorldPtr & world_,
                                   osgShadow::MinimalShadowMap * msm) :
    dart::gui::osg::WorldNode(world_, msm) {

    mInterface = new DracoInterface(); mSensorData = new DracoSensorData();
    mCommand = new DracoCommand();

    mSkel = world_->getSkeleton("Draco");
    mDof = mSkel->getNumDofs();
    mTorqueCommand = Eigen::VectorXd::Zero(mDof);
    YAML::Node simulation_cfg =
        YAML::LoadFile(THIS_COM"Config/Draco/SIMULATION.yaml");
    myUtils::readParameter(simulation_cfg, "release_time", mReleaseTime);
}

DracoWorldNode::~DracoWorldNode() {}

void DracoWorldNode::customPreStep() {
    mSensorData->q = mSkel->getPositions();
    mSensorData->qdot = mSkel->getVelocities();
    mSensorData->jtrq = mSkel->getForces();

    mInterface->getCommand(mSensorData, mCommand);
    mTorqueCommand.tail(mDof - 6) = mCommand->jtrq;

    //static int count(0);
    //if (count*SERVO_RATE < mReleaseTime) {
        //_fixXY();
        //_fixRP();
    //}
    //count++;

    //mTorqueCommand.setZero();
    mSkel->setForces(mTorqueCommand);
}

void DracoWorldNode::_fixXY() {
    double kp(1500.); double kd(150.);
    Eigen::VectorXd q = mSkel->getPositions();
    Eigen::VectorXd qdot = mSkel->getVelocities();
    for (int i = 0; i < 2; ++i) {
        mTorqueCommand[i] = - (kp * q[i]) - (kd * qdot[i]);
    }
}

void DracoWorldNode::_fixRP() {
    double kp(5.0); double kd(0.5);
    Eigen::VectorXd q = mSkel->getPositions();
    Eigen::VectorXd qdot = mSkel->getVelocities();
    for (int i = 4; i < 6; ++i) {
        mTorqueCommand[i] = - (kp * q[i]) - (kd * qdot[i]);
    }
}
