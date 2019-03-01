#include <Configuration.h>
#include <PnC/FixedDracoPnC/FixedDracoInterface.hpp>
#include <Simulator/Dart/FixedDraco/FixedDracoWorldNode.hpp>
#include <Utils/IO/IOUtilities.hpp>

FixedDracoWorldNode::FixedDracoWorldNode(
    const dart::simulation::WorldPtr& _world, osgShadow::MinimalShadowMap* msm)
    : dart::gui::osg::WorldNode(_world, msm),
      count_(0),
      t_(0.0),
      servo_rate_(0) {
    try {
        YAML::Node simulation_cfg =
            YAML::LoadFile(THIS_COM "Config/FixedDraco/SIMULATION.yaml");
        myUtils::readParameter(simulation_cfg, "servo_rate", servo_rate_);
    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
    }
    world_ = _world;
    mSkel = world_->getSkeleton("Draco");
    mGround = world_->getSkeleton("ground_skeleton");
    mTorqueCommand = Eigen::VectorXd::Zero(10);

    mInterface = new FixedDracoInterface();

    mSensorData = new FixedDracoSensorData();
    mSensorData->q = Eigen::VectorXd::Zero(10);
    mSensorData->qdot = Eigen::VectorXd::Zero(10);

    mCommand = new FixedDracoCommand();
    mCommand->jtrq = Eigen::VectorXd::Zero(10);
}

FixedDracoWorldNode::~FixedDracoWorldNode() {
    delete mInterface;
    delete mSensorData;
    delete mCommand;
}

void FixedDracoWorldNode::customPreStep() {
    t_ = (double)count_ * servo_rate_;

    mSensorData->q = mSkel->getPositions();
    mSensorData->qdot = mSkel->getVelocities();

    mInterface->getCommand(mSensorData, mCommand);

    mTorqueCommand = mCommand->jtrq;

    // mTorqueCommand.setZero();
    mSkel->setForces(mTorqueCommand);

    count_++;
}
