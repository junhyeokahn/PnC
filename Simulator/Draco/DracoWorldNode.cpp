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

    mSensorData->q = mSkel->getPositions().tail(10);
    mSensorData->qdot = mSkel->getVelocities().tail(10);
    mSensorData->jtrq = mSkel->getForces().tail(10);

    _get_imu_data(mSensorData->imu_ang_vel, mSensorData->imu_acc);
    _check_foot_contact(mSensorData->rfoot_contact, mSensorData->lfoot_contact);

    mInterface->getCommand(mSensorData, mCommand);
    mTorqueCommand.tail(10) = mCommand->jtrq;

    // Low level position control
    for (int i = 0; i < 10; ++i) {
        mTorqueCommand[i+6] += mKp[i] * (mCommand->q[i] - mSensorData->q[i]) +
            mKd[i] * (mCommand->qdot[i] - mSensorData->qdot[i]);
    }

    // hold xy
    _hold_xy();

    //mTorqueCommand.setZero(); //TODO
    mSkel->setForces(mTorqueCommand);

}

void DracoWorldNode::_get_imu_data( Eigen::VectorXd & ang_vel,
                                    Eigen::VectorXd & acc) {
    Eigen::VectorXd ang_vel_local = mSkel->getBodyNode("torso")->getSpatialVelocity(dart::dynamics::Frame::World(), mSkel->getBodyNode("torso")).head(3);
    ang_vel = ang_vel_local;
    Eigen::MatrixXd rot_world_torso(3, 3);
    rot_world_torso = mSkel->getBodyNode("torso")->getWorldTransform().linear();
    Eigen::Vector3d global_grav(0, 0, 9.81);
    Eigen::Vector3d local_grav = rot_world_torso.transpose() * global_grav;
    acc = local_grav;
}

void DracoWorldNode::_check_foot_contact( bool & rfoot_contact,
                                          bool & lfoot_contact) {
    Eigen::VectorXd rfoot_pos = mSkel->getBodyNode("rAnkle")->getCOM();
    Eigen::VectorXd lfoot_pos = mSkel->getBodyNode("lAnkle")->getCOM();
    //std::cout << rfoot_pos << std::endl;
    //std::cout << lfoot_pos << std::endl;
    //exit(0);
    if(  fabs(lfoot_pos[2]) < 0.029){
        lfoot_contact = true;
        //printf("left contact\n");
    }else { lfoot_contact = false; }
    if (fabs(rfoot_pos[2])<0.029){
        rfoot_contact = true;
        //printf("right contact\n");
    } else { rfoot_contact = false; }
}

void DracoWorldNode::_hold_xy() {
    static int count_(0);

    static double des_x = (mSkel->getPositions())[0];
    static double des_y = (mSkel->getPositions())[1];
    double act_x = mSkel->getPositions()[0];
    double act_xdot = mSkel->getVelocities()[0];
    double act_y = mSkel->getPositions()[1];
    double act_ydot = mSkel->getVelocities()[1];

    if ((double)count_*SERVO_RATE < mReleaseTime) {
        mTorqueCommand[0] = 1000 * (des_x - act_x)
            -50 * act_xdot ;
        mTorqueCommand[1] = 1000 * (- act_y)
            -50 * act_ydot ;
    } else {
        static bool first__ = true;
        if (first__) {
            std::cout << "[Release]" << std::endl;
            first__ = false;
        }
    }
    count_++;
}
