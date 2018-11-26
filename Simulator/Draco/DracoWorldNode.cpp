#include "DracoWorldNode.hpp"
#include "PnC/DracoPnC/DracoInterface.hpp"
#include "Utils/DataManager.hpp"
#include "Utils/Utilities.hpp"
#include "Utils/ParamHandler.hpp"
#include "Configuration.h"

DracoWorldNode::DracoWorldNode(const dart::simulation::WorldPtr & _world,
                                   osgShadow::MinimalShadowMap * msm) :
    dart::gui::osg::WorldNode(_world, msm) {

    world_ = _world;

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
    mSensorData->rfoot_contact = false;
    mSensorData->lfoot_contact = false;

    mCommand = new DracoCommand();
    mCommand->turn_off = false;
    mCommand->q = Eigen::VectorXd::Zero(10);
    mCommand->qdot = Eigen::VectorXd::Zero(10);
    mCommand->jtrq = Eigen::VectorXd::Zero(10);

    mSkel = world_->getSkeleton("Draco");
    mGround = world_->getSkeleton("ground_skeleton");
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

    DataManager* data_manager = DataManager::GetDataManager();
    q_sim_ = Eigen::VectorXd::Zero(16);
    data_manager->RegisterData(&q_sim_, VECT, "q_sim", 16);
}

DracoWorldNode::~DracoWorldNode() {}

void DracoWorldNode::customPreStep() {

    mSensorData->q = mSkel->getPositions().tail(10);
    q_sim_ = mSkel->getPositions();
    mSensorData->qdot = mSkel->getVelocities().tail(10);
    mSensorData->jtrq = mSkel->getForces().tail(10);

    _get_imu_data(mSensorData->imu_ang_vel, mSensorData->imu_acc);
    _check_foot_contact(mSensorData->rfoot_contact, mSensorData->lfoot_contact);
    //_check_collision();

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
    if (fabs(lfoot_pos[2]) < 0.0252){
        lfoot_contact = true;
        //printf("left contact\n");
    } else {
        lfoot_contact = false;
    }
    if (fabs(rfoot_pos[2])<0.0252){
        rfoot_contact = true;
        //printf("right contact\n");
    } else {
        rfoot_contact = false;
    }
}

void DracoWorldNode::_hold_xy() {
    static int count_(0);

    static double des_x = (mSkel->getPositions())[0];
    static double des_y = (mSkel->getPositions())[1];
    static double des_xdot(0.);
    static double des_ydot(0.);

    double act_x = mSkel->getPositions()[0];
    double act_xdot = mSkel->getVelocities()[0];
    double act_y = mSkel->getPositions()[1];
    double act_ydot = mSkel->getVelocities()[1];

    double t = (double)count_*SERVO_RATE;

    if (t < mReleaseTime) {

        if (mSensorData->rfoot_contact && mSensorData->lfoot_contact) {
            static double interp_init_time = t;
            static double ini_des_x = des_x;
            static double ini_des_y = des_y;
            static double final_des_x =
                ( (mSkel->getBodyNode("rAnkle")->getCOM())[0] + (mSkel->getBodyNode("lAnkle")->getCOM())[0] ) / 2.0;
            static double final_des_y =
                ( (mSkel->getBodyNode("rAnkle")->getCOM())[1] + (mSkel->getBodyNode("lAnkle")->getCOM())[1] ) / 2.0;
            des_x = myUtils::smooth_changing(ini_des_x, final_des_x, mReleaseTime - interp_init_time, t - interp_init_time);
            des_y = myUtils::smooth_changing(ini_des_y, final_des_y, mReleaseTime - interp_init_time, t - interp_init_time);

            des_x -= 0.068;
        }

        mTorqueCommand[0] = 1500 * (des_x - act_x)
            +100 * (des_xdot - act_xdot) ;
        mTorqueCommand[1] = 1500 * (des_y - act_y)
            +100 * (des_ydot - act_ydot) ;
    } else {
        static bool first__ = true;
        if (first__) {
            std::cout << "[Release]" << std::endl;
            first__ = false;
        }
    }


    count_++;
}

void DracoWorldNode::_check_collision() {
    auto collisionEngine = world_->getConstraintSolver()->getCollisionDetector();
    auto groundCol = collisionEngine->createCollisionGroup(mGround.get());
    auto robotCol = collisionEngine->createCollisionGroup(mSkel.get());
    dart::collision::CollisionOption option;
    dart::collision::CollisionResult result;
    bool collision = groundCol->collide(robotCol.get(), option, &result);
    auto colliding_body_nodes_list = result.getCollidingBodyNodes();
    std::cout << "collision size" << std::endl;
    std::cout << colliding_body_nodes_list.size() << std::endl;
}
