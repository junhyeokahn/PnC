#include <Configuration.h>
#include <PnC/DracoPnC/DracoInterface.hpp>
#include <Simulator/Dart/Draco/DracoLedPosAnnouncer.hpp>
#include <Simulator/Dart/Draco/DracoWorldNode.hpp>
#include <Utils/IO/DataManager.hpp>
#include <Utils/IO/IOUtilities.hpp>

DracoWorldNode::DracoWorldNode(const dart::simulation::WorldPtr& _world,
                               osgShadow::MinimalShadowMap* msm, int mpi_idx,
                               int env_idx)
    : dart::gui::osg::WorldNode(_world, msm),
      count_(0),
      t_(0.0),
      servo_rate_(0.001),
      mpi_idx_(mpi_idx),
      env_idx_(env_idx) {
    world_ = _world;
    mSkel = world_->getSkeleton("Draco");
    mGround = world_->getSkeleton("ground_skeleton");
    mStar = world_->getSkeleton("star");
    mTorus = world_->getSkeleton("torus");
    mDof = mSkel->getNumDofs();
    mTorqueCommand = Eigen::VectorXd::Zero(mDof);
    b_parallel_ = true;

    SetParameters_();

    led_pos_announcer_ = new DracoLedPosAnnouncer();
    led_pos_announcer_->start();
    UpdateLedData_();

    mInterface = new DracoInterface(mpi_idx, env_idx);

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
    mSensorData->rfoot_ati = Eigen::VectorXd::Zero(6);
    mSensorData->lfoot_ati = Eigen::VectorXd::Zero(6);
    mSensorData->rfoot_contact = false;
    mSensorData->lfoot_contact = false;

    mCommand = new DracoCommand();
    mCommand->turn_off = false;
    mCommand->q = Eigen::VectorXd::Zero(10);
    mCommand->qdot = Eigen::VectorXd::Zero(10);
    mCommand->jtrq = Eigen::VectorXd::Zero(10);

    DataManager* data_manager = DataManager::GetDataManager();
    q_sim_ = Eigen::VectorXd::Zero(16);
    data_manager->RegisterData(&q_sim_, VECT, "q_sim", 16);
}

DracoWorldNode::DracoWorldNode(const dart::simulation::WorldPtr& _world,
                               osgShadow::MinimalShadowMap* msm)
    : dart::gui::osg::WorldNode(_world, msm),
      count_(0),
      t_(0.0),
      servo_rate_(0.001),
      mpi_idx_(0),
      env_idx_(0) {
    world_ = _world;
    mSkel = world_->getSkeleton("Draco");
    mGround = world_->getSkeleton("ground_skeleton");
    mStar = world_->getSkeleton("star");
    mTorus = world_->getSkeleton("torus");
    mDof = mSkel->getNumDofs();
    mTorqueCommand = Eigen::VectorXd::Zero(mDof);
    b_parallel_ = false;

    SetParameters_();

    led_pos_announcer_ = new DracoLedPosAnnouncer();
    led_pos_announcer_->start();
    UpdateLedData_();

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
    mSensorData->rfoot_ati = Eigen::VectorXd::Zero(6);
    mSensorData->lfoot_ati = Eigen::VectorXd::Zero(6);
    mSensorData->rfoot_contact = false;
    mSensorData->lfoot_contact = false;

    mCommand = new DracoCommand();
    mCommand->turn_off = false;
    mCommand->q = Eigen::VectorXd::Zero(10);
    mCommand->qdot = Eigen::VectorXd::Zero(10);
    mCommand->jtrq = Eigen::VectorXd::Zero(10);

    DataManager* data_manager = DataManager::GetDataManager();
    q_sim_ = Eigen::VectorXd::Zero(16);
    data_manager->RegisterData(&q_sim_, VECT, "q_sim", 16);
}

DracoWorldNode::~DracoWorldNode() {
    delete mInterface;
    delete mSensorData;
    delete mCommand;
    delete led_pos_announcer_;
}

void DracoWorldNode::SetParameters_() {
    try {
        YAML::Node simulation_cfg =
            YAML::LoadFile(THIS_COM "Config/Draco/SIMULATION.yaml");
        myUtils::readParameter(simulation_cfg, "servo_rate", servo_rate_);
        myUtils::readParameter(simulation_cfg, "release_time", mReleaseTime);
        myUtils::readParameter(simulation_cfg, "check_collision",
                               b_check_collision_);
        myUtils::readParameter(simulation_cfg, "show_viewer", b_show_viewer_);
        myUtils::readParameter(simulation_cfg, "display_target_frame",
                               b_plot_target_);
        myUtils::readParameter(simulation_cfg, "plot_guided_foot",
                               b_plot_guided_foot_);
        myUtils::readParameter(simulation_cfg, "plot_adjusted_foot",
                               b_plot_adjusted_foot_);
        myUtils::readParameter(simulation_cfg, "camera_manipulator",
                               b_camera_manipulator_);
        myUtils::readParameter(simulation_cfg, "print_computation_time",
                               b_print_computation_time);
        YAML::Node control_cfg = simulation_cfg["control_configuration"];
        myUtils::readParameter(control_cfg, "kp", mKp);
        myUtils::readParameter(control_cfg, "kd", mKd);
        if (!b_parallel_) b_show_viewer_ = true;

        if (!b_show_viewer_) {
            b_plot_target_ = false;
            b_plot_guided_foot_ = false;
            b_plot_adjusted_foot_ = false;
            b_camera_manipulator_ = false;
        }
    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
    }
}
void DracoWorldNode::customPreStep() {
    t_ = (double)count_ * servo_rate_;

    mSensorData->q = mSkel->getPositions().tail(10);
    q_sim_ = mSkel->getPositions();
    mSensorData->qdot = mSkel->getVelocities().tail(10);
    mSensorData->jtrq = mSkel->getForces().tail(10);

    UpdateLedData_();
    _get_imu_data(mSensorData->imu_ang_vel, mSensorData->imu_acc);
    _check_foot_contact(mSensorData->rfoot_contact, mSensorData->lfoot_contact);

    if (b_plot_target_) PlotTargetLocation_();
    if (b_plot_guided_foot_) PlotGuidedFootLocation_();
    if (b_plot_adjusted_foot_) PlotAdjustedFootLocation_();
    if (b_camera_manipulator_) UpdateCameraPos_();

    if (b_check_collision_) {
        _check_collision();
    }
    //_get_ati_data();

    if (b_print_computation_time) {
        clock_.start();
    }

    mInterface->getCommand(mSensorData, mCommand);

    if (b_print_computation_time) {
        printf("time: %f\n", clock_.stop());
    }

    // Low level FeedForward and Position Control
    mTorqueCommand.tail(10) = mCommand->jtrq;
    for (int i = 0; i < 10; ++i) {
        mTorqueCommand[i + 6] +=
            mKp[i] * (mCommand->q[i] - mSensorData->q[i]) +
            mKd[i] * (mCommand->qdot[i] - mSensorData->qdot[i]);
    }
    mTorqueCommand.head(6).setZero();

    // hold robot at the initial phase
    if (t_ < mReleaseTime) {
        _hold_xy();
        _hold_rot();
    } else {
        static bool first__ = true;
        if (first__) {
            std::cout << "[Release]" << std::endl;
            first__ = false;
        }
    }

    // mTorqueCommand.setZero();
    mSkel->setForces(mTorqueCommand);

    count_++;
}

void DracoWorldNode::UpdateLedData_() {
    for (int led_idx(0); led_idx < NUM_MARKERS; ++led_idx) {
        led_pos_announcer_->msg.visible[led_idx] = 1;
        for (int axis_idx(0); axis_idx < 3; ++axis_idx) {
            led_pos_announcer_->msg.data[3 * led_idx + axis_idx] =
                mSkel
                    ->getBodyNode(
                        led_pos_announcer_->led_link_idx_list[led_idx])
                    ->getTransform()
                    .translation()[axis_idx] *
                1000.0;
        }
    }
}

void DracoWorldNode::_get_imu_data(Eigen::VectorXd& ang_vel,
                                   Eigen::VectorXd& acc) {
    //Eigen::VectorXd ang_vel_local =
        //mSkel->getBodyNode("Torso")
            //->getSpatialVelocity(dart::dynamics::Frame::World(),
                                 //mSkel->getBodyNode("Torso"))
            //.head(3);

    //ang_vel = ang_vel_local;
    //Eigen::MatrixXd rot_world_torso(3, 3);
    //rot_world_torso = mSkel->getBodyNode("Torso")->getWorldTransform().linear();
    //Eigen::Vector3d global_grav(0, 0, 9.81);
    //Eigen::Vector3d local_grav = rot_world_torso.transpose() * global_grav;
    //acc = local_grav;
    //// TEST
    //Eigen::VectorXd torso_angvel_local =
        //mSkel->getBodyNode("Torso")
            //->getSpatialVelocity(dart::dynamics::Frame::World(),
                                 //mSkel->getBodyNode("Torso")).head(3);
    //Eigen::MatrixXd rot_world_torso(3, 3);
    //rot_world_torso = mSkel->getBodyNode("Torso")->getWorldTransform().linear();
    //Eigen::Vector3d grav_g(0, 0, 9.81);
    //Eigen::Vector3d torso_acc_local= rot_world_torso.transpose() * grav_g;
    //myUtils::pretty_print(torso_angvel_local, std::cout, "torso_angvel_local");
    //myUtils::pretty_print(torso_acc_local, std::cout, "torso_acc_local");
    //// TEST END

    Eigen::VectorXd ang_vel_local =
        mSkel->getBodyNode("IMU")
            ->getSpatialVelocity(dart::dynamics::Frame::World(),
                                 mSkel->getBodyNode("IMU"))
            .head(3);

    ang_vel = ang_vel_local;
    Eigen::MatrixXd R_world_imu(3, 3);
    R_world_imu = mSkel->getBodyNode("IMU")->getWorldTransform().linear();
    Eigen::Vector3d global_grav(0, 0, 9.81);
    Eigen::Vector3d local_grav = R_world_imu.transpose() * global_grav;
    acc = local_grav;
}

void DracoWorldNode::_check_foot_contact(bool& rfoot_contact,
                                         bool& lfoot_contact) {
    Eigen::VectorXd r_c = mSkel->getBodyNode("rFootCenter")->getCOM();
    Eigen::VectorXd l_c = mSkel->getBodyNode("lFootCenter")->getCOM();
    Eigen::VectorXd r_f = mSkel->getBodyNode("rFootFront")->getCOM();
    Eigen::VectorXd l_f = mSkel->getBodyNode("lFootFront")->getCOM();
    Eigen::VectorXd r_b = mSkel->getBodyNode("rFootBack")->getCOM();
    Eigen::VectorXd l_b = mSkel->getBodyNode("lFootBack")->getCOM();

    if ((fabs(l_c[2]) < 0.002) || (fabs(l_f[2]) < 0.002) ||
        (fabs(l_b[2] < 0.002))) {
        lfoot_contact = true;
        // printf("left contact\n");
    } else {
        lfoot_contact = false;
    }

    if ((fabs(r_c[2]) < 0.002) || (fabs(r_f[2]) < 0.002) ||
        (fabs(r_b[2] < 0.002))) {
        rfoot_contact = true;
        // printf("right contact\n");
    } else {
        rfoot_contact = false;
    }
}

void DracoWorldNode::_hold_rot() {
    Eigen::VectorXd q = mSkel->getPositions();
    Eigen::VectorXd v = mSkel->getVelocities();
    double kp(200);
    double kd(5);
    mTorqueCommand[3] = kp * (-q[3]) + kd * (-v[3]);
    mTorqueCommand[4] = kp * (-q[4]) + kd * (-v[4]);
    mTorqueCommand[5] = kp * (-q[5]) + kd * (-v[5]);
}

void DracoWorldNode::_hold_xy() {
    static double des_x = (mSkel->getPositions())[0];
    static double des_y = (mSkel->getPositions())[1];
    static double des_xdot(0.);
    static double des_ydot(0.);

    Eigen::VectorXd q = mSkel->getPositions();
    Eigen::VectorXd v = mSkel->getVelocities();

    double kp(1500);
    double kd(100);

    mTorqueCommand[0] = kp * (des_x - q[0]) + kd * (des_xdot - v[0]);
    mTorqueCommand[1] = kp * (des_y - q[1]) + kd * (des_ydot - v[1]);
}

void DracoWorldNode::_check_collision() {
    auto collisionEngine =
        world_->getConstraintSolver()->getCollisionDetector();
    auto groundCol = collisionEngine->createCollisionGroup(mGround.get());
    auto robotCol = collisionEngine->createCollisionGroup(mSkel.get());
    dart::collision::CollisionOption option;
    dart::collision::CollisionResult result;
    bool collision = groundCol->collide(robotCol.get(), option, &result);
    auto colliding_body_nodes_list = result.getCollidingBodyNodes();

    for (auto bn : colliding_body_nodes_list) {
        if (t_ > mReleaseTime && bn->getName() == "Torso") {
            std::cout << "Torso Collision Happen" << std::endl;
            exit(0);
        }
    }
}

void DracoWorldNode::PlotTargetLocation_() {
    dart::dynamics::SimpleFramePtr frame =
        world_->getSimpleFrame("target_frame");
    Eigen::Isometry3d tf = ((DracoInterface*)mInterface)->GetTargetIso();
    frame->setTransform(tf);
}

void DracoWorldNode::UpdateCameraPos_() {
    Eigen::Isometry3d torso_iso = mSkel->getBodyNode("Torso")->getTransform();
    Eigen::Vector3d torso_vec = torso_iso.translation();
    mViewer->getCameraManipulator()->setHomePosition(
        ::osg::Vec3(torso_vec[0] + 2, torso_vec[1] - 6., torso_vec[2] + 2),
        ::osg::Vec3(torso_vec[0], torso_vec[1], torso_vec[2]),
        ::osg::Vec3(0.0, 0.0, 1.0));
    mViewer->setCameraManipulator(mViewer->getCameraManipulator());
}

void DracoWorldNode::PlotGuidedFootLocation_() {
    Eigen::VectorXd q = Eigen::VectorXd::Zero(6);
    q[5] = 0.001;
    Eigen::Vector3d guided_foot =
        ((DracoInterface*)mInterface)->GetGuidedFoot();
    q[3] = guided_foot[0];
    q[4] = guided_foot[1];
    mTorus->setPositions(q);
    mTorus->setVelocities(Eigen::VectorXd::Zero(6));
}

void DracoWorldNode::PlotAdjustedFootLocation_() {
    Eigen::VectorXd q = Eigen::VectorXd::Zero(6);
    q[5] = 0.001;
    Eigen::Vector3d adjusted_foot =
        ((DracoInterface*)mInterface)->GetAdjustedFoot();
    q[3] = adjusted_foot[0];
    q[4] = adjusted_foot[1];
    mStar->setPositions(q);
    mStar->setVelocities(Eigen::VectorXd::Zero(6));
}
