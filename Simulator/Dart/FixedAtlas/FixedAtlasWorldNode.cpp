#include <Configuration.h>
#include <PnC/FixedAtlasPnC/FixedAtlasInterface.hpp>
#include <Simulator/Dart/FixedAtlas/FixedAtlasWorldNode.hpp>
#include <Utils/IO/IOUtilities.hpp>

FixedAtlasWorldNode::FixedAtlasWorldNode(
    const dart::simulation::WorldPtr& _world, osgShadow::MinimalShadowMap* msm)
    : dart::gui::osg::WorldNode(_world, msm),
      count_(0),
      t_(0.0),
      servo_rate_(0) {
    try {
        YAML::Node simulation_cfg =
            YAML::LoadFile(THIS_COM "Config/FixedAtlas/SIMULATION.yaml");
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

    mInterface = new FixedAtlasInterface();

    mSensorData = new FixedAtlasSensorData();
    mSensorData->q = Eigen::VectorXd::Zero(30);
    mSensorData->qdot = Eigen::VectorXd::Zero(30);

    mCommand = new FixedAtlasCommand();
    mCommand->q = Eigen::VectorXd::Zero(30);
    mCommand->qdot = Eigen::VectorXd::Zero(30);
    mCommand->jtrq = Eigen::VectorXd::Zero(30);
}

FixedAtlasWorldNode::~FixedAtlasWorldNode() {
    delete mInterface;
    delete mSensorData;
    delete mCommand;
}

void FixedAtlasWorldNode::customPreStep() {
    t_ = (double)count_ * servo_rate_;

    //std::cout << "============" << std::endl; 
    //std::cout << mSkel->getPositions() << std::endl;
    //std::cout << "============" << std::endl; 
    //std::cout << mSkel->getVelocities() << std::endl;


    mSensorData->q = mSkel->getPositions();
    mSensorData->qdot = mSkel->getVelocities();

    mInterface->getCommand(mSensorData, mCommand);

    mTorqueCommand = mCommand->jtrq;
    
    //std::cout << "============" << std::endl; 
    //std::cout << count_ << "trq command" << std::endl;
    //std::cout << mTorqueCommand << std::endl;
    //std::exit(0);
    // mTorqueCommand.setZero();
    mSkel->setForces(mTorqueCommand);
    myUtils::saveVector(mSensorData->q, "q");
    myUtils::saveVector(mCommand->q, "q_des");
    myUtils::saveVector(mSensorData->qdot, "qdot");
    myUtils::saveVector(mCommand->qdot, "qdot_des");
    myUtils::saveVector(mTorqueCommand, "gamma");

    count_++;
}
