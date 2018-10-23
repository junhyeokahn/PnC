#include "DracoWorldNode.hpp"
#include "PnC/DracoPnC/DracoInterface.hpp"
#include "Utils/DataManager.hpp"
#include "Utils/Utilities.hpp"
#include "Utils/ParamHandler.hpp"
#include "Configuration.h"

DracoWorldNode::DracoWorldNode(const dart::simulation::WorldPtr & world_,
                                   osgShadow::MinimalShadowMap * msm) :
    dart::gui::osg::WorldNode(world_, msm) {

    mInterface = new DracoInterface();

    mSensorData = new DracoSensorData();
    mSensorData->imu_ang_vel = Eigen::VectorXd::Zero(3);
    mSensorData->imu_acc = Eigen::VectorXd::Zero(3);
    mSensorData->q = Eigen::VectorXd::Zero(10);
    mSensorData->qdot = Eigen::VectorXd::Zero(10);
    mSensorData->jtrq = Eigen::VectorXd::Zero(10);
    mSensorData->temperature = Eigen::VectorXd::Zero(10);
    mSensorData->motor_current = Eigen::VectorXd::Zero(10);
    mSensorData->bus_voltage = Eigen::VectorXd::Zero(10);
    mSensorData->bus_current = Eigen::VectorXd::Zero(10);
    mSensorData->rotor_inertia = Eigen::VectorXd::Zero(10);
    mSensorData->rfoot_contact = true;
    mSensorData->lfoot_contact = true;

    mCommand = new DracoCommand();
    mCommand->turn_off = false;
    mCommand->q = Eigen::VectorXd::Zero(10);
    mCommand->qdot = Eigen::VectorXd::Zero(10);
    mCommand->jtrq = Eigen::VectorXd::Zero(10);

    mSkel = world_->getSkeleton("Draco");
    mDof = mSkel->getNumDofs();
    mTorqueCommand = Eigen::VectorXd::Zero(mDof);
    try {
        YAML::Node simulation_cfg =
            YAML::LoadFile(THIS_COM"Config/Draco/SIMULATION.yaml");
        myUtils::readParameter(simulation_cfg, "release_time", mReleaseTime);
        YAML::Node control_cfg = simulation_cfg["control_configuration"];
        myUtils::readParameter(control_cfg, "kp", mKp);
        myUtils::readParameter(control_cfg, "kd", mKd);
    } catch(std::runtime_error& e) {
        std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
    }
}

DracoWorldNode::~DracoWorldNode() {}

void DracoWorldNode::customPreStep() {
    static int count_(0);
    static double initial_x = (mSkel->getPositions())[0];
    static double initial_y = (mSkel->getPositions())[1];

    mSensorData->q = mSkel->getPositions().tail(10);
    mSensorData->qdot = mSkel->getVelocities().tail(10);
    mSensorData->jtrq = mSkel->getForces().tail(10);

    _get_imu_data(mSensorData->imu_ang_vel, mSensorData->imu_acc);

    mInterface->getCommand(mSensorData, mCommand);
    mTorqueCommand.tail(10) = mCommand->jtrq;

    // Low level position control
    for (int i = 0; i < 10; ++i) {
        mTorqueCommand[i+6] += mKp[i] * (mCommand->q[i] - mSensorData->q[i]) +
            mKd[i] * (mCommand->qdot[i] - mSensorData->qdot[i]);
    }

    // hold xy
    if (count_*SERVO_RATE < mReleaseTime) {
        mTorqueCommand[0] = 1000 * (initial_x - (mSkel->getPositions())[0])
            -50 * (mSkel->getVelocities())[0] ;
        mTorqueCommand[1] = 1000 * (initial_y - (mSkel->getPositions())[1])
            -50 * (mSkel->getVelocities())[1] ;
    } else {

    }

    //mTorqueCommand.setZero();
    mSkel->setForces(mTorqueCommand);

    count_++;
}

void DracoWorldNode::_get_imu_data( Eigen::VectorXd & ang_vel,
                                    Eigen::VectorXd & acc) {
    ang_vel = mSkel->getBodyNode("torso")->getSpatialVelocity(dart::dynamics::Frame::World(), dart::dynamics::Frame::World()).head(3);
    Eigen::MatrixXd rot_world_torso(3, 3);
    rot_world_torso = mSkel->getBodyNode("torso")->getWorldTransform().linear();
    Eigen::Vector3d global_grav(0, 0, -9.81);
    Eigen::Vector3d local_grav = rot_world_torso.transpose() * global_grav;
    acc = local_grav;
    //std::cout << "Acc" << std::endl;
    //std::cout << acc << std::endl;
    //std::cout << "Ang Vel" << std::endl;
    //std::cout << ang_vel << std::endl;
}
