#include <PnC/DracoPnC/DracoInterface.hpp>
#include <ADE/DracoSim.hpp>
#include <thread>

void DracoSim::displayJointFrames(const dart::simulation::WorldPtr& world,
                                      const dart::dynamics::SkeletonPtr& robot) {
    for (std::size_t i = 0; i < robot->getNumBodyNodes(); ++i) {
        dart::dynamics::BodyNode* bn = robot->getBodyNode(i);
        for (std::size_t j = 0; j < bn->getNumChildJoints(); ++j) {
            const dart::dynamics::Joint* joint = bn->getChildJoint(j);
            const Eigen::Isometry3d offset =
                    joint->getTransformFromParentBodyNode();

            dart::gui::osg::InteractiveFramePtr frame =
                    std::make_shared<dart::gui::osg::InteractiveFrame>(
                            bn, joint->getName() + "/frame", offset);

            for (const auto type : {dart::gui::osg::InteractiveTool::ANGULAR,
                                    dart::gui::osg::InteractiveTool::PLANAR})
                for (std::size_t i = 0; i < 3; ++i)
                    frame->getTool(type, i)->setEnabled(false);

            world->addSimpleFrame(frame);
        }
    }
}

void DracoSim::addTargetFrame(const dart::simulation::WorldPtr& world) {
    dart::gui::osg::InteractiveFramePtr frame =
            std::make_shared<dart::gui::osg::InteractiveFrame>(
                    dart::dynamics::Frame::World(), "target_frame");
    for (int i = 0; i < 3; ++i) {
        frame->getTool(dart::gui::osg::InteractiveTool::PLANAR, i)
                ->setEnabled(false);
        frame->getTool(dart::gui::osg::InteractiveTool::ANGULAR, i)
                ->setEnabled(false);
    }
    world->addSimpleFrame(frame);
}

void DracoSim::_setJointLimitConstraint(dart::dynamics::SkeletonPtr robot) {
    for (int i = 0; i < robot->getNumJoints(); ++i) {
        dart::dynamics::Joint* joint = robot->getJoint(i);
        // std::cout << i << "th" << std::endl;
        // std::cout << joint->isPositionLimitEnforced() << std::endl;
        joint->setPositionLimitEnforced(true);
    }
}

void DracoSim::_setInitialConfiguration(dart::dynamics::SkeletonPtr robot) {
    int lKneeIdx = robot->getDof("lKnee")->getIndexInSkeleton();
    int lHipPitchIdx = robot->getDof("lHipPitch")->getIndexInSkeleton();
    int rKneeIdx = robot->getDof("rKnee")->getIndexInSkeleton();
    int rHipPitchIdx = robot->getDof("rHipPitch")->getIndexInSkeleton();
    int lAnkleIdx = robot->getDof("lAnkle")->getIndexInSkeleton();
    int rAnkleIdx = robot->getDof("rAnkle")->getIndexInSkeleton();

    int initPos(1);  // 0 : Home, 1 : Simulation, 2 : Experiment
    Eigen::VectorXd q = robot->getPositions();

    switch (initPos) {
        case 1: {
            q[2] = 0.9;
            double alpha(-M_PI / 4.);
            double beta(M_PI / 5.5);
            q[lHipPitchIdx] = alpha;
            q[lKneeIdx] = beta - alpha;
            q[rHipPitchIdx] = alpha;
            q[rKneeIdx] = beta - alpha;
            q[lAnkleIdx] = M_PI / 2 - beta;
            q[rAnkleIdx] = M_PI / 2 - beta;
            break;
        }
    }

    robot->setPositions(q);
}

DracoSim::DracoSim(DracoInterface* interface) {

    // =========================================================================
    // Parse Yaml for Simulator
    // =========================================================================
    bool isRecord;
    bool b_display_joint_frame;
    bool b_display_target_frame;
    bool b_joint_limit_enforced;
    double servo_rate;
    try {
        YAML::Node simulation_cfg =
                YAML::LoadFile(THIS_COM "Config/Draco/SIMULATION.yaml");
        myUtils::readParameter(simulation_cfg, "is_record", isRecord);
        myUtils::readParameter(simulation_cfg, "display_joint_frame",
                               b_display_joint_frame);
        myUtils::readParameter(simulation_cfg, "display_target_frame",
                               b_display_target_frame);
        myUtils::readParameter(simulation_cfg, "joint_limit_enforced",
                               b_joint_limit_enforced);
        myUtils::readParameter(simulation_cfg, "servo_rate", servo_rate);
    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
    }

    // =========================================================================
    // Generate world and add skeletons
    // =========================================================================
    dart::simulation::WorldPtr world(new dart::simulation::World);
    dart::utils::DartLoader urdfLoader;
    dart::dynamics::SkeletonPtr ground = urdfLoader.parseSkeleton(
            THIS_COM "RobotModel/Ground/ground_terrain.urdf");
    dart::dynamics::SkeletonPtr robot = urdfLoader.parseSkeleton(
            THIS_COM "RobotModel/Robot/Draco/DracoSim_Dart.urdf");
    world->addSkeleton(ground);
    world->addSkeleton(robot);

    // =========================================================================
    // Friction & Restitution Coefficient
    // =========================================================================
    double friction(10.);
    double restit(0.0);
    ground->getBodyNode("ground_link")->setFrictionCoeff(friction);
    robot->getBodyNode("Torso")->setFrictionCoeff(friction);
    ground->getBodyNode("ground_link")->setRestitutionCoeff(restit);
    robot->getBodyNode("Torso")->setRestitutionCoeff(restit);
    robot->getBodyNode("rAnkle")->setFrictionCoeff(friction);
    robot->getBodyNode("lAnkle")->setFrictionCoeff(friction);
    robot->getBodyNode("lAnkle")->setRestitutionCoeff(restit);
    robot->getBodyNode("rAnkle")->setRestitutionCoeff(restit);

    Eigen::Vector3d gravity(0.0, 0.0, -9.81);
    world->setGravity(gravity);
    world->setTimeStep(servo_rate);

    // =========================================================================
    // Display Joints Frame
    // =========================================================================
    if (b_display_joint_frame) displayJointFrames(world, robot);

    // =========================================================================
    // Display Target Frame
    // =========================================================================
    if (b_display_target_frame) addTargetFrame(world);

    // =========================================================================
    // Initial configuration
    // =========================================================================
    _setInitialConfiguration(robot);

    // =========================================================================
    // Enabel Joit Limits
    // =========================================================================
    if (b_joint_limit_enforced) _setJointLimitConstraint(robot);

    // =========================================================================
    // Wrap a worldnode
    // =========================================================================
    world_ = new DracoWorldNode(world, interface);
    world_->setNumStepsPerCycle(30);
}

void DracoSim::StartSim() {
    exit = false;
    // =========================================================================
    // Create and Set Viewer
    // =========================================================================
    //dart::gui::osg::Viewer viewer;
    viewer.addWorldNode(world_);
    viewer.simulate(true);
    viewer.switchHeadlights(false);
    ::osg::Vec3 p1(1.0, 0.2, 1.0);
    p1 = p1 * 0.7;
    viewer.getLightSource(0)->getLight()->setPosition(
            ::osg::Vec4(p1[0], p1[1], p1[2], 0.0));
    viewer.getCamera()->setClearColor(osg::Vec4(0.93f, 0.95f, 1.0f, 0.95f));
    viewer.getCamera()->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // viewer.setUpViewInWindow(0, 0, 2880, 1800);
    viewer.setUpViewInWindow(1440, 0, 500, 500);
    viewer.getCameraManipulator()->setHomePosition(
            ::osg::Vec3(5.14, 2.28, 3.0) * 1.5, ::osg::Vec3(0.0, 0.2, 0.5),
            ::osg::Vec3(0.0, 0.0, 1.0));
    viewer.setCameraManipulator(viewer.getCameraManipulator());
    thread_ = std::thread(&DracoSim::StartThread, this);
}

void DracoSim::StopSim() {
    exit = true;
    thread_.join();
}

void DracoSim::StartThread() {
    while(!viewer.done() && !exit.load()) {
        viewer.frame();
    }
}

