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
    try {
        YAML::Node simulation_cfg =
            YAML::LoadFile(THIS_COM"Config/Draco/SIMULATION.yaml");
        YAML::Node control_cfg = simulation_cfg["control_configuration"];
        myUtils::readParameter(control_cfg, "kp", mKp);
        myUtils::readParameter(control_cfg, "kd", mKd);
    } catch(std::runtime_error& e) {
        std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
    }
}

DracoWorldNode::~DracoWorldNode() {}

void DracoWorldNode::customPreStep() {
    mSensorData->q = mSkel->getPositions();
    mSensorData->qdot = mSkel->getVelocities();
    mSensorData->jtrq = mSkel->getForces();

    mInterface->getCommand(mSensorData, mCommand);
    mTorqueCommand.tail(mDof - 6) = mCommand->jtrq;

    // Low level position control
    for (int i = 0; i < 10; ++i) {
        mTorqueCommand[i+6] += mKp[i] * (mCommand->q[i+6] - mSensorData->q[i+6]) +
            mKd[i] * (mCommand->qdot[i+6] - mSensorData->qdot[i+6]);
    }
    //mTorqueCommand.setZero();
    mSkel->setForces(mTorqueCommand);
}
