#include <Configuration.h>
#include <PnC/AtlasPnC/AtlasInterface.hpp>
#include <Simulator/Dart/Atlas/AtlasWorldNode.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>

AtlasWorldNode::AtlasWorldNode(const dart::simulation::WorldPtr& _world)
    : dart::gui::osg::WorldNode(_world), count_(0), t_(0.0), servo_rate_(0) {
    world_ = _world;
    robot_ = world_->getSkeleton("multisense_sl");
    trq_lb_ = robot_->getForceLowerLimits();
    trq_ub_ = robot_->getForceUpperLimits();
    n_dof_ = robot_->getNumDofs();
    ground_ = world_->getSkeleton("ground_skeleton");

    star_ = world_->getSkeleton("star");
    torus_ = world_->getSkeleton("torus");

    trq_cmd_ = Eigen::VectorXd::Zero(n_dof_);
    b_parallel_ = false;

    interface_ = new AtlasInterface();
    sensor_data_ = new AtlasSensorData();
    command_ = new AtlasCommand();

    SetParams_();
}

AtlasWorldNode::AtlasWorldNode(const dart::simulation::WorldPtr& _world,
                               int mpi_idx, int env_idx)
    : dart::gui::osg::WorldNode(_world), count_(0), t_(0.0), servo_rate_(0) {
    world_ = _world;
    robot_ = world_->getSkeleton("multisense_sl");
    trq_lb_ = robot_->getForceLowerLimits();
    trq_ub_ = robot_->getForceUpperLimits();
    n_dof_ = robot_->getNumDofs();
    ground_ = world_->getSkeleton("ground_skeleton");

    star_ = world_->getSkeleton("star");
    torus_ = world_->getSkeleton("torus");

    trq_cmd_ = Eigen::VectorXd::Zero(n_dof_);
    b_parallel_ = true;

    interface_ = new AtlasInterface(mpi_idx, env_idx);
    sensor_data_ = new AtlasSensorData();
    command_ = new AtlasCommand();

    SetParams_();
}

AtlasWorldNode::~AtlasWorldNode() {
    delete interface_;
    delete sensor_data_;
    delete command_;
}

void AtlasWorldNode::customPreStep() {
    t_ = (double)count_ * servo_rate_;
    if (dart::math::isNan(robot_->getPositions())) {
        std::cout << "NaN" << std::endl;
        myUtils::pretty_print(robot_->getPositions(), std::cout, "q");
        // exit(0);
    }

    sensor_data_->q = robot_->getPositions().tail(n_dof_ - 6);
    sensor_data_->qdot = robot_->getVelocities().tail(n_dof_ - 6);
    GetImuData_(sensor_data_->imu_ang_vel, sensor_data_->imu_acc);
    GetContactSwitchData_(sensor_data_->rfoot_contact,
                          sensor_data_->lfoot_contact);

    interface_->getCommand(sensor_data_, command_);

    trq_cmd_.tail(n_dof_ - 6) = command_->jtrq;
    for (int i = 0; i < n_dof_ - 6; ++i) {
        trq_cmd_[i + 6] += kp_ * (command_->q[i] - sensor_data_->q[i]) +
                           kd_ * (command_->qdot[i] - sensor_data_->qdot[i]);
    }
    trq_cmd_.head(6).setZero();
    // if (count_ < 10) HoldXY_();

    // trq_cmd_.setZero();
    Eigen::VectorXd clipped_trq =
        myUtils::CropVector(trq_cmd_, trq_lb_, trq_ub_, "final trq");
    robot_->setForces(clipped_trq);

    count_++;

    if (b_show_target_frame_) PlotTargetLocation_();
    if (b_manipulate_camera_) ManipulateCameraPos_();
    if (b_plot_guided_foot_) PlotGuidedFootLocation_();
    if (b_plot_adjusted_foot_) PlotAdjustedFootLocation_();
}

void AtlasWorldNode::PlotGuidedFootLocation_() {
    Eigen::VectorXd q = Eigen::VectorXd::Zero(6);
    q[5] = 0.001;
    Eigen::Vector3d guided_foot =
        ((AtlasInterface*)interface_)->GetGuidedFoot();
    q[3] = guided_foot[0];
    q[4] = guided_foot[1];
    torus_->setPositions(q);
    torus_->setVelocities(Eigen::VectorXd::Zero(6));
}

void AtlasWorldNode::PlotAdjustedFootLocation_() {
    Eigen::VectorXd q = Eigen::VectorXd::Zero(6);
    q[5] = 0.001;
    Eigen::Vector3d adjusted_foot =
        ((AtlasInterface*)interface_)->GetAdjustedFoot();
    q[3] = adjusted_foot[0];
    q[4] = adjusted_foot[1];
    star_->setPositions(q);
    star_->setVelocities(Eigen::VectorXd::Zero(6));
}

void AtlasWorldNode::GetImuData_(Eigen::VectorXd& ang_vel,
                                 Eigen::VectorXd& acc) {
    Eigen::VectorXd ang_vel_local =
        robot_->getBodyNode("pelvis")
            ->getSpatialVelocity(dart::dynamics::Frame::World(),
                                 robot_->getBodyNode("pelvis"))
            .head(3);
    ang_vel = ang_vel_local;
    Eigen::MatrixXd rot_wp(3, 3);
    rot_wp = robot_->getBodyNode("pelvis")->getWorldTransform().linear();
    Eigen::VectorXd global_grav(3);
    global_grav << 0, 0, 9.81;
    Eigen::VectorXd local_grav = rot_wp.transpose() * global_grav;
    acc = local_grav;
}

void AtlasWorldNode::GetContactSwitchData_(bool& rfoot_contact,
                                           bool& lfoot_contact) {
    Eigen::VectorXd rf =
        robot_->getBodyNode("r_sole")->getWorldTransform().translation();
    Eigen::VectorXd lf =
        robot_->getBodyNode("l_sole")->getWorldTransform().translation();

    // myUtils::pretty_print(rf, std::cout, "right_sole");
    // myUtils::pretty_print(lf, std::cout, "left_sole");

    if (fabs(rf[2] < 0.01)) {
        rfoot_contact = true;
        // printf("right contact\n");
    } else {
        rfoot_contact = false;
    }

    if (fabs(lf[2] < 0.01)) {
        lfoot_contact = true;
        // printf("left contact\n");
    } else {
        lfoot_contact = false;
    }
}

void AtlasWorldNode::HoldXY_() {
    Eigen::VectorXd q = robot_->getPositions();
    Eigen::VectorXd v = robot_->getVelocities();
    double kp(200);
    double kd(50);
    double des_x(0.);
    double des_xdot(0.);
    for (int i = 0; i < 2; ++i)
        trq_cmd_[i] = kp * (des_x - q[i]) + kd * (des_xdot - v[i]);
}

void AtlasWorldNode::PlotTargetLocation_() {
    dart::dynamics::SimpleFramePtr frame =
        world_->getSimpleFrame("target_frame");
    Eigen::Isometry3d tf = ((AtlasInterface*)interface_)->GetTargetIso();
    frame->setTransform(tf);
}

void AtlasWorldNode::ManipulateCameraPos_() {
    Eigen::Isometry3d pelvis_iso =
        robot_->getBodyNode("utorso")->getTransform();
    Eigen::Vector3d pelvis_vec = pelvis_iso.translation();
    mViewer->getCameraManipulator()->setHomePosition(
        ::osg::Vec3(pelvis_vec[0] + 3, pelvis_vec[1] - 8., pelvis_vec[2] + 3),
        ::osg::Vec3(pelvis_vec[0], pelvis_vec[1], pelvis_vec[2]),
        ::osg::Vec3(0.0, 0.0, 1.0));
    mViewer->setCameraManipulator(mViewer->getCameraManipulator());
}

void AtlasWorldNode::SetParams_() {
    try {
        YAML::Node simulation_cfg =
            YAML::LoadFile(THIS_COM "Config/Atlas/SIMULATION.yaml");
        myUtils::readParameter(simulation_cfg, "servo_rate", servo_rate_);
        myUtils::readParameter(simulation_cfg, "show_target_frame",
                               b_show_target_frame_);
        myUtils::readParameter(simulation_cfg, "camera_manipulator",
                               b_manipulate_camera_);
        myUtils::readParameter(simulation_cfg, "show_viewer", b_show_viewer_);
        myUtils::readParameter(simulation_cfg["control_configuration"], "kp",
                              kp_);
        myUtils::readParameter(simulation_cfg["control_configuration"], "kd",
                               kd_);
        myUtils::readParameter(simulation_cfg, "plot_guided_foot",
                               b_plot_guided_foot_);
        myUtils::readParameter(simulation_cfg, "plot_adjusted_foot",
                               b_plot_adjusted_foot_);

        if (!b_parallel_) b_show_viewer_ = true;
        if (!b_show_viewer_) {
            b_manipulate_camera_ = false;
            b_show_target_frame_ = false;
        }

    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
    }
}

