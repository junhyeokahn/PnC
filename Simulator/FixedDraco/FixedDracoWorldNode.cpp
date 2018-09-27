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
    try {
        YAML::Node simulation_cfg =
            YAML::LoadFile(THIS_COM"Config/FixedDraco/SIMULATION.yaml");
        YAML::Node control_cfg = simulation_cfg["control_configuration"];
        myUtils::readParameter(control_cfg, "kp", mKp);
        myUtils::readParameter(control_cfg, "kd", mKd);
    } catch(std::runtime_error& e) {
        std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
    }
}

FixedDracoWorldNode::~FixedDracoWorldNode() {
    delete mInterface;
    delete mCommand;
    delete mSensorData;
}

void FixedDracoWorldNode::customPreStep() {
    mSensorData->q = mSkel->getPositions();
    mSensorData->qdot = mSkel->getVelocities();
    mSensorData->jtrq = mSkel->getForces();

    mInterface->getCommand(mSensorData, mCommand);
    mTorqueCommand = mCommand->jtrq;

    // Inv Kin Test
    //mTorqueCommand.setZero();
    //mSkel->setPositions(mCommand->q);
    //mSkel->setVelocities(mCommand->qdot);
    //mSkel->setPositions(init_q);

    // Low level position control
    for (int i = 0; i < 10; ++i) {
        mTorqueCommand[i] += mKp[i] * (mCommand->q[i] - mSensorData->q[i]) +
            mKd[i] * (mCommand->qdot[i] - mSensorData->qdot[i]);
    }
    //mTorqueCommand.setZero();
    mSkel->setForces(mTorqueCommand);
}
