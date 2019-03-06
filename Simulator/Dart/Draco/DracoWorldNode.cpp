#include <Configuration.h>
#include <PnC/DracoPnC/DracoInterface.hpp>
#include <Simulator/Dart/Draco/DracoLedPosAnnouncer.hpp>
#include <Simulator/Dart/Draco/DracoWorldNode.hpp>
#include <Utils/IO/DataManager.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <cassert>

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
    mDof = mSkel->getNumDofs();
    mTorqueCommand = Eigen::VectorXd::Zero(mDof);
    try {
        YAML::Node simulation_cfg =
            YAML::LoadFile(THIS_COM "Config/Draco/SIMULATION.yaml");
        myUtils::readParameter(simulation_cfg, "servo_rate", servo_rate_);
        myUtils::readParameter(simulation_cfg, "release_time", mReleaseTime);
        myUtils::readParameter(simulation_cfg, "check_collision",
                               b_check_collision_);
        myUtils::readParameter(simulation_cfg, "print_computation_time",
                               b_print_computation_time);
        YAML::Node control_cfg = simulation_cfg["control_configuration"];
        myUtils::readParameter(control_cfg, "kp", mKp);
        myUtils::readParameter(control_cfg, "kd", mKd);
    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
    }

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
    mDof = mSkel->getNumDofs();
    mTorqueCommand = Eigen::VectorXd::Zero(mDof);
    try {
        YAML::Node simulation_cfg =
            YAML::LoadFile(THIS_COM "Config/Draco/SIMULATION.yaml");
        myUtils::readParameter(simulation_cfg, "servo_rate", servo_rate_);
        myUtils::readParameter(simulation_cfg, "release_time", mReleaseTime);
        myUtils::readParameter(simulation_cfg, "check_collision",
                               b_check_collision_);
        myUtils::readParameter(simulation_cfg, "print_computation_time",
                               b_print_computation_time);
        YAML::Node control_cfg = simulation_cfg["control_configuration"];
        myUtils::readParameter(control_cfg, "kp", mKp);
        myUtils::readParameter(control_cfg, "kd", mKd);
    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
    }

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

void DracoWorldNode::customPreStep() {
    t_ = (double)count_ * servo_rate_;

    mSensorData->q = mSkel->getPositions().tail(10);
    q_sim_ = mSkel->getPositions();
    mSensorData->qdot = mSkel->getVelocities().tail(10);
    mSensorData->jtrq = mSkel->getForces().tail(10);

    UpdateLedData_();
    _get_imu_data(mSensorData->imu_ang_vel, mSensorData->imu_acc);
    _check_foot_contact(mSensorData->rfoot_contact, mSensorData->lfoot_contact);
    UpdateTargetLocation_();

    if (b_check_collision_) {
        _check_collision();
    }
    GetFTSensorData_();

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
    Eigen::VectorXd ang_vel_local =
        mSkel->getBodyNode("Torso")
            ->getSpatialVelocity(dart::dynamics::Frame::World(),
                                 mSkel->getBodyNode("Torso"))
            .head(3);
    ang_vel = ang_vel_local;
    Eigen::MatrixXd rot_world_torso(3, 3);
    rot_world_torso = mSkel->getBodyNode("Torso")->getWorldTransform().linear();
    Eigen::Vector3d global_grav(0, 0, 9.81);
    Eigen::Vector3d local_grav = rot_world_torso.transpose() * global_grav;
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

void DracoWorldNode::GetFTSensorData_() {
    dart::dynamics::BodyNode* lankle_bn = mSkel->getBodyNode("lAnkle");
    dart::dynamics::BodyNode* rankle_bn = mSkel->getBodyNode("rAnkle");
    const dart::collision::CollisionResult& _result =
        world_->getLastCollisionResult();

    Eigen::VectorXd lfoot_ati = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd rfoot_ati = Eigen::VectorXd::Zero(6);

    for (const auto& contact : _result.getContacts()) {
        for (const auto& shapeNode :
             lankle_bn->getShapeNodesWith<dart::dynamics::CollisionAspect>()) {
            if (shapeNode == contact.collisionObject1->getShapeFrame() ||
                shapeNode == contact.collisionObject2->getShapeFrame()) {
                double normal(contact.normal(2));
                Eigen::VectorXd w_c = Eigen::VectorXd::Zero(6);
                w_c.tail(3) = contact.force * normal;
                Eigen::Isometry3d T_wc = Eigen::Isometry3d::Identity();
                T_wc.translation() = contact.point;
                Eigen::Isometry3d T_wa =
                    mSkel->getBodyNode("lAnkle")->getTransform(
                        dart::dynamics::Frame::World());
                Eigen::Isometry3d T_ca = T_wc.inverse() * T_wa;
                Eigen::MatrixXd AdT_ca = dart::math::getAdTMatrix(T_ca);
                Eigen::VectorXd w_a = Eigen::VectorXd::Zero(6);
                w_a = AdT_ca.transpose() * w_c;
                lfoot_ati += w_a;
            }
        }

        for (const auto& shapeNode :
             rankle_bn->getShapeNodesWith<dart::dynamics::CollisionAspect>()) {
            if (shapeNode == contact.collisionObject1->getShapeFrame() ||
                shapeNode == contact.collisionObject2->getShapeFrame()) {
                double normal(contact.normal(2));
                Eigen::VectorXd w_c = Eigen::VectorXd::Zero(6);
                w_c.tail(3) = contact.force * normal;
                Eigen::Isometry3d T_wc = Eigen::Isometry3d::Identity();
                T_wc.translation() = contact.point;
                Eigen::Isometry3d T_wa =
                    mSkel->getBodyNode("rAnkle")->getTransform(
                        dart::dynamics::Frame::World());
                Eigen::Isometry3d T_ca = T_wc.inverse() * T_wa;
                Eigen::MatrixXd AdT_ca = dart::math::getAdTMatrix(T_ca);
                Eigen::VectorXd w_a = Eigen::VectorXd::Zero(6);
                w_a = AdT_ca.transpose() * w_c;
                rfoot_ati += w_a;
            }
        }
    }

    mSensorData->lfoot_ati = lfoot_ati;
    mSensorData->rfoot_ati = rfoot_ati;

    // std::cout << "--------------------------------------------" << std::endl;
    // myUtils::pretty_print(lfoot_ati, std::cout, "lfoot ati");
    // myUtils::pretty_print(rfoot_ati, std::cout, "rfoot ati");
}

void DracoWorldNode::UpdateTargetLocation_() {
    dart::dynamics::SimpleFramePtr frame =
        world_->getSimpleFrame("target_frame");
    Eigen::Isometry3d tf = ((DracoInterface*)mInterface)->GetTargetIso();
    frame->setTransform(tf);
}
