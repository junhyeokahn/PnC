#include <Configuration.h>
#include <PnC/AtlasPnC/AtlasInterface.hpp>
#include <Simulator/Dart/Atlas/AtlasWorldNode.hpp>
#include <Utils/IO/IOUtilities.hpp>

AtlasWorldNode::AtlasWorldNode(
    const dart::simulation::WorldPtr& _world, osgShadow::MinimalShadowMap* msm)
    : dart::gui::osg::WorldNode(_world, msm),
      count_(0),
      t_(0.0),
      servo_rate_(0) {
    try {
        YAML::Node simulation_cfg =
            YAML::LoadFile(THIS_COM "Config/Atlas/SIMULATION.yaml");
        myUtils::readParameter(simulation_cfg, "servo_rate", servo_rate_);
    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
    }
    world_ = _world;
    mSkel = world_->getSkeleton("multisense_sl");
    mGround = world_->getSkeleton("ground_skeleton");
    mTorqueCommand = Eigen::VectorXd::Zero(30); //Find Function to get the number of the active joint (Numdof)??

    mInterface = new AtlasInterface();

    mSensorData = new AtlasSensorData();
    mSensorData->q = Eigen::VectorXd::Zero(30);
    mSensorData->qdot = Eigen::VectorXd::Zero(30);

    mCommand = new AtlasCommand();
    mCommand->jtrq = Eigen::VectorXd::Zero(30);
}

AtlasWorldNode::~AtlasWorldNode() {
    delete mInterface;
    delete mSensorData;
    delete mCommand;
}

void AtlasWorldNode::customPreStep() {
    t_ = (double)count_ * servo_rate_;

    mSensorData->q = mSkel->getPositions();
    mSensorData->qdot = mSkel->getVelocities();

    mInterface->getCommand(mSensorData, mCommand);

    mTorqueCommand = mCommand->jtrq;

    // mTorqueCommand.setZero();
    mSkel->setForces(mTorqueCommand);

    count_++;
}
