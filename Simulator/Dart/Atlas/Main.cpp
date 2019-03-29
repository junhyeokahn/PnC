#include <Configuration.h>
#include <Simulator/Dart/Atlas/AtlasWorldNode.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>

void displayJointFrames(const dart::simulation::WorldPtr& world,
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

void addTargetFrame(const dart::simulation::WorldPtr& world) {
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

class OneStepProgress : public osgGA::GUIEventHandler {
   public:
    OneStepProgress(AtlasWorldNode* worldnode) : worldnode_(worldnode) {}

    /** Deprecated, Handle events, return true if handled, false otherwise. */
    virtual bool handle(const osgGA::GUIEventAdapter& ea,
                        osgGA::GUIActionAdapter& /*aa*/) {
        if (ea.getEventType() == osgGA::GUIEventAdapter::KEYUP) {
            if (ea.getKey() == 'f') {
                int numStepProgress(50);
                for (int i = 0; i < numStepProgress; ++i) {
                    worldnode_->customPreStep();
                    worldnode_->getWorld()->step();
                    worldnode_->customPostStep();
                }
                return true;
            }
        }
        return false;
    }
    AtlasWorldNode* worldnode_;
};

void _setJointLimitConstraint(dart::dynamics::SkeletonPtr robot) {
    for (int i = 0; i < robot->getNumJoints(); ++i) {
        dart::dynamics::Joint* joint = robot->getJoint(i);
        // std::cout << i << "th" << std::endl;
        // std::cout << joint->isPositionLimitEnforced() << std::endl;
        joint->setPositionLimitEnforced(true);
    }
}

void _printRobotModel(dart::dynamics::SkeletonPtr robot) {
    // for (int i = 0; i < robot->getNumBodyNodes(); ++i) {
    // dart::dynamics::BodyNodePtr bn = robot->getBodyNode(i);
    // std::cout << i << "th" << std::endl;
    // std::cout << bn->getName() << std::endl;
    // std::cout << bn->getMass() << std::endl;
    //}

    // for (int i = 0; i < robot->getNumJoints(); ++i) {
    // dart::dynamics::Joint* joint = robot->getJoint(i);
    // std::cout << i << "th" << std::endl;
    // std::cout << joint->getNumDofs() << std::endl;
    //}

    for (int i = 0; i < robot->getNumDofs(); ++i) {
        dart::dynamics::DegreeOfFreedom* dof = robot->getDof(i);
        std::cout << i << "th" << std::endl;
        std::cout << "dof name : " << dof->getName() << std::endl;
        std::cout << "child body node name and mass : "
                  << dof->getChildBodyNode()->getName() << " , "
                  << dof->getChildBodyNode()->getMass() << std::endl;
    }

    // std::cout << "num dof" << std::endl;
    // std::cout << robot->getNumDofs() << std::endl;
    // std::cout << robot->getNumJoints() << std::endl;
    // std::cout << "mass mat row" << std::endl;
    // std::cout << robot->getMassMatrix().rows() << std::endl;
    // std::cout << robot->getMassMatrix().cols() << std::endl;
    // std::cout << "q" << std::endl;
    // std::cout << robot->getPositions() << std::endl;
    // std::cout << "robot total mass" << std::endl;
    // std::cout << robot->getMass() << std::endl;
    // std::cout << "robot position" << std::endl;
    // std::cout << robot->getPositions() << std::endl;

    exit(0);
}

void _setInitialConfiguration(dart::dynamics::SkeletonPtr robot) {
    double height, yaw;
    try {
        YAML::Node simulation_cfg =
            YAML::LoadFile(THIS_COM "Config/Atlas/SIMULATION.yaml");
        myUtils::readParameter(simulation_cfg["initial_configuration"],
                               "height", height);
        myUtils::readParameter(simulation_cfg["initial_configuration"], "yaw",
                               yaw);
    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
    }
    int l_leg_hpy = robot->getDof("l_leg_hpy")->getIndexInSkeleton();
    int r_leg_hpy = robot->getDof("r_leg_hpy")->getIndexInSkeleton();
    int l_leg_kny = robot->getDof("l_leg_kny")->getIndexInSkeleton();
    int r_leg_kny = robot->getDof("r_leg_kny")->getIndexInSkeleton();
    int l_leg_aky = robot->getDof("l_leg_aky")->getIndexInSkeleton();
    int r_leg_aky = robot->getDof("r_leg_aky")->getIndexInSkeleton();

    int r_arm_shx = robot->getDof("r_arm_shx")->getIndexInSkeleton();
    int r_arm_ely = robot->getDof("r_arm_ely")->getIndexInSkeleton();
    int r_arm_elx = robot->getDof("r_arm_elx")->getIndexInSkeleton();
    int l_arm_shx = robot->getDof("l_arm_shx")->getIndexInSkeleton();
    int l_arm_ely = robot->getDof("l_arm_ely")->getIndexInSkeleton();
    int l_arm_elx = robot->getDof("l_arm_elx")->getIndexInSkeleton();

    Eigen::VectorXd q = robot->getPositions();
    q[2] = height;
    q[3] = yaw;
    q[l_leg_hpy] = -0.4;
    q[r_leg_hpy] = -0.4;
    q[l_leg_kny] = 0.8;
    q[r_leg_kny] = 0.8;
    q[l_leg_aky] = -0.4;
    q[r_leg_aky] = -0.4;

    q[r_arm_shx] = 0.8;
    q[r_arm_ely] = -0.9;
    q[r_arm_elx] = 1.8;
    q[l_arm_shx] = -0.8;
    q[l_arm_ely] = -0.9;
    q[l_arm_elx] = -1.8;
    robot->setPositions(q);
}

int main(int argc, char** argv) {
    double servo_rate;
    bool b_show_joint_frame;
    bool b_show_target_frame;
    bool b_show;
    try {
        YAML::Node simulation_cfg =
            YAML::LoadFile(THIS_COM "Config/Atlas/SIMULATION.yaml");
        myUtils::readParameter(simulation_cfg, "servo_rate", servo_rate);
        myUtils::readParameter(simulation_cfg, "show_joint_frame",
                               b_show_joint_frame);
        myUtils::readParameter(simulation_cfg, "show_target_frame",
                               b_show_target_frame);
        myUtils::readParameter(simulation_cfg, "show_viewer", b_show);
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
        THIS_COM "RobotModel/Robot/Atlas/AtlasSim_Dart.urdf");
    dart::dynamics::SkeletonPtr star =
        urdfLoader.parseSkeleton(THIS_COM "RobotModel/Object/star.urdf");
    dart::dynamics::SkeletonPtr torus =
        urdfLoader.parseSkeleton(THIS_COM "RobotModel/Object/torus.urdf");

    world->addSkeleton(ground);
    world->addSkeleton(robot);
    world->addSkeleton(star);
    world->addSkeleton(torus);

    // =========================================================================
    // Friction & Restitution Coefficient
    // =========================================================================
    double friction(0.9);
    double restit(0.0);
    ground->getBodyNode("ground_link")->setFrictionCoeff(friction);
    robot->getBodyNode("r_foot")->setFrictionCoeff(friction);
    robot->getBodyNode("l_foot")->setFrictionCoeff(friction);

    Eigen::Vector3d gravity(0.0, 0.0, -9.81);
    world->setGravity(gravity);
    world->setTimeStep(servo_rate);

    // =========================================================================
    // Display Joints Frame
    // =========================================================================
    if (b_show_joint_frame) displayJointFrames(world, robot);

    // =========================================================================
    // Display Target Frame
    // =========================================================================
    if (b_show_target_frame) addTargetFrame(world);

    // =========================================================================
    // Initial configuration
    // =========================================================================
    _setInitialConfiguration(robot);

    // =========================================================================
    // Enabel Joit Limits
    // =========================================================================
    //_setJointLimitConstraint(robot);

    // =========================================================================
    // Print Model Info
    // =========================================================================
    //_printRobotModel(robot);

    // =========================================================================
    // Wrap a worldnode
    // =========================================================================
    osg::ref_ptr<AtlasWorldNode> node;
    if (argc > 1) {
        node =
            new AtlasWorldNode(world, std::stoi(argv[1]), std::stoi(argv[2]));
    } else {
        node = new AtlasWorldNode(world);
        b_show = true;
    }
    node->setNumStepsPerCycle(30);

    // =========================================================================
    // Create and Set Viewer
    // =========================================================================
    if (b_show) {
        dart::gui::osg::Viewer viewer;
        viewer.addWorldNode(node);
        viewer.simulate(false);
        viewer.switchHeadlights(false);
        ::osg::Vec3 p1(1.0, 0.2, 1.0);
        p1 = p1 * 0.7;
        viewer.getLightSource(0)->getLight()->setPosition(
            ::osg::Vec4(p1[0], p1[1], p1[2], 0.0));
        viewer.getCamera()->setClearColor(osg::Vec4(0.93f, 0.95f, 1.0f, 0.95f));
        viewer.getCamera()->setClearMask(GL_COLOR_BUFFER_BIT |
                                         GL_DEPTH_BUFFER_BIT);
        viewer.addEventHandler(new OneStepProgress(node));

        // viewer.setUpViewInWindow(0, 0, 2880, 1800);
        viewer.setUpViewInWindow(1440, 0, 500, 500);
        viewer.getCameraManipulator()->setHomePosition(
            ::osg::Vec3(5.14, 2.28, 3.0) * 1.5, ::osg::Vec3(0.0, 0.2, 0.5),
            ::osg::Vec3(0.0, 0.0, 1.0));
        viewer.setCameraManipulator(viewer.getCameraManipulator());
        viewer.run();
    } else {
        while (true) {
            node->customPreStep();
            node->getWorld()->step();
            node->customPostStep();
        }
    }
}
