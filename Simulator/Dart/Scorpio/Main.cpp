#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>
#include <Configuration.h>
#include <Simulator/Dart/Scorpio/ScorpioWorldNode.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <PnC/DracoPnC/DracoInterface.hpp>
#include <PnC/ScorpioPnC/ScorpioInterface.hpp>

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
    OneStepProgress(ScorpioWorldNode* worldnode) : worldnode_(worldnode) {}
    // Reachability node
    //OneStepProgress(ScorpioWorldNodeReach* worldnode) : worldnode_(worldnode) {}

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
     ScorpioWorldNode* worldnode_;
    // ScorpioWorldNodeReach* worldnode_;  // reachability node
};

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

    // for (int i = 0; i < robot->getNumDofs(); ++i) {
    // dart::dynamics::DegreeOfFreedom* dof = robot->getDof(i);
    // std::cout << i << "th" << std::endl;
    // std::cout << dof->getName() << std::endl;
    // std::cout << "child body node name : " <<
    // dof->getChildBodyNode()->getName() << std::endl; std::cout <<
    // dof->getCoulombFriction() << std::endl;
    //}

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
    std::cout << "robot position" << std::endl;
    std::cout << robot->getPositions() << std::endl;
}

void _setInitialConfiguration(dart::dynamics::SkeletonPtr robot, Eigen::VectorXd & q_init) {

    Eigen::VectorXd conf_init(robot->getNumDofs());
    conf_init.setZero();

    int mode_init(1);

    switch(mode_init){
        case 0:{
            conf_init.setZero();
            break;
        }
        case 1:{
            conf_init[0] = q_init[0];
            conf_init[1] = q_init[1];
            conf_init[2] = -q_init[1];
            conf_init[3] = q_init[1];
            conf_init[4] = q_init[2];
            conf_init[5] = q_init[3];
            conf_init[6] = -q_init[3];
            conf_init[7] = q_init[3];
            conf_init[8] = q_init[4];
            break;
        }
        default:{
            conf_init.setZero();
            break;
        }
    }

    robot->setPositions(conf_init);

    std::cout<<"======================================================"<<std::endl;
    std::cout<<"***** joint initialized, DOF: "<< robot->getNumDofs()<<std::endl;
    std::cout<<"(1) Yaw joint 1: "<< robot->getDof("joint1")->getIndexInSkeleton() <<std::endl;
    std::cout<<"(2) Elev1 joint 1: "<< robot->getDof("joint2")->getIndexInSkeleton() <<std::endl;
    std::cout<<"(3) Elev1 joint 2: "<< robot->getDof("joint3")->getIndexInSkeleton() <<std::endl;
    std::cout<<"(4) Elev1 joitn 3: "<< robot->getDof("joint4")->getIndexInSkeleton() <<std::endl;
    std::cout<<"(5) Yaw joint 2: "<< robot->getDof("joint5")->getIndexInSkeleton() <<std::endl;
    std::cout<<"(6) Elev2 joint 1: "<< robot->getDof("joint6")->getIndexInSkeleton() <<std::endl;
    std::cout<<"(7) Elev2 joint 2: "<< robot->getDof("joint7")->getIndexInSkeleton() <<std::endl;
    std::cout<<"(8) Elev2 joint 3: "<< robot->getDof("joint8")->getIndexInSkeleton() <<std::endl;
    std::cout<<"(9) Wrist Skew : "<< robot->getDof("joint9")->getIndexInSkeleton() <<std::endl;
    std::cout<<"(10) Wrist Pitch: "<< robot->getDof("joint10")->getIndexInSkeleton() <<std::endl;
    std::cout<<"(11) Wrist Roll: " << robot->getDof("joint11")->getIndexInSkeleton() <<std::endl;
    Eigen::Vector3d end_pos_init = robot->getBodyNode("end_effector")->getTransform().translation();
    myUtils::pretty_print(end_pos_init, std::cout , "<1> end effector pos");
    std::cout<<"======================================================"<<std::endl;

}

void _setInitialConfiguration(dart::dynamics::SkeletonPtr robot) {
    int lKneeIdx = robot->getDof("lKnee")->getIndexInSkeleton();
    int lHipPitchIdx = robot->getDof("lHipPitch")->getIndexInSkeleton();
    int rKneeIdx = robot->getDof("rKnee")->getIndexInSkeleton();
    int rHipPitchIdx = robot->getDof("rHipPitch")->getIndexInSkeleton();
    int lAnkleIdx = robot->getDof("lAnkle")->getIndexInSkeleton();
    int rAnkleIdx = robot->getDof("rAnkle")->getIndexInSkeleton();

    int initPos(1);  // 0 : Home, 1 : Simulation, 2 : Experiment
    Eigen::VectorXd q = robot->getPositions();

    switch (initPos) {
        case 0: {
            q[2] = 1.425;
            q[lAnkleIdx] = 0.;
            q[rAnkleIdx] = 0.;
            // q[lAnkleIdx] = M_PI/2;
            // q[rAnkleIdx] = M_PI/2;
            break;
        }
        case 1: {
            //q[0] = 2.5;
            //q[3] = M_PI;
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
        case 2: {
            YAML::Node simulation_cfg =
                YAML::LoadFile(THIS_COM "Config/Draco/SIMULATION.yaml");
            double hanging_height(0.0);
            myUtils::readParameter(simulation_cfg, "hanging_height",
                                   hanging_height);
            Eigen::VectorXd init_config;
            myUtils::readParameter(simulation_cfg, "initial_configuration",
                                   init_config);
            q[2] = hanging_height;
            q.tail(10) = init_config;
            break;
        }
        default:
            std::cout << "[wrong initial pos case] in Draco/Main.cpp"
                      << std::endl;
    }

    robot->setPositions(q);
}
void _setInitialConfiguration_2(dart::dynamics::SkeletonPtr robot) {
    Eigen::VectorXd q = robot->getPositions();
    q[0] = 1.3918;
    q[1] = -0.360352;
    q[2] = 0.52563;
    robot->setPositions(q);
}
void _SetMeshColorURDF(dart::dynamics::SkeletonPtr robot){
	for(size_t i=0; i < robot->getNumBodyNodes(); ++i)
	{
	  dart::dynamics::BodyNode* bn = robot->getBodyNode(i);
	  for(size_t j=0; j < bn->getNumShapeNodes(); ++j)
	  {
	    dart::dynamics::ShapeNode* sn = bn->getShapeNode(j);
	    if(sn->getVisualAspect())
	    {
	      dart::dynamics::MeshShape* ms =
		  dynamic_cast<dart::dynamics::MeshShape*>(sn->getShape().get());
	      if(ms)
		ms->setColorMode(dart::dynamics::MeshShape::SHAPE_COLOR);
	    }
	  }
	}
}

void _setJointLimitConstraint(dart::dynamics::SkeletonPtr robot) {
    for (int i = 0; i < robot->getNumJoints(); ++i) {
        dart::dynamics::Joint* joint = robot->getJoint(i);
        // std::cout << i << "th" << std::endl;
        // std::cout << joint->isPositionLimitEnforced() << std::endl;
        joint->setPositionLimitEnforced(true);
    }
}

void _SetJointConstraint(dart::simulation::WorldPtr & world, dart::dynamics::SkeletonPtr robot){
    dart::dynamics::BodyNode* bd1 = robot->getBodyNode("link1");
    dart::dynamics::BodyNode* bd2 = robot->getBodyNode("link4_end");
   Eigen::Vector3d offset(0.09, 0.1225, -0.034975);
    Eigen::Vector3d joint_pos1 = bd1->getTransform()*offset;
    dart::constraint::BallJointConstraintPtr cl1 =
        std::make_shared<dart::constraint::BallJointConstraint>(bd1,bd2,joint_pos1);

    Eigen::Vector3d pos_b1 = bd1->getTransform().translation();
    Eigen::Vector3d pos_b2 = bd2->getTransform().translation();

    dart::dynamics::BodyNode* bd3 = robot->getBodyNode("link5");
    dart::dynamics::BodyNode* bd4 = robot->getBodyNode("link8_end");
    Eigen::Vector3d joint_pos2 = bd3->getTransform()*offset;
    dart::constraint::BallJointConstraintPtr cl2 =
        std::make_shared<dart::constraint::BallJointConstraint>(bd3,bd4,joint_pos2);

    world->getConstraintSolver()->addConstraint(cl1);
    world->getConstraintSolver()->addConstraint(cl2);

    std::cout<<"======================================================"<<std::endl;
    std::cout<<"***** ball joint constraints are initialzed." <<std::endl;
    myUtils::pretty_print(pos_b1, std::cout, "pos bd1");
    myUtils::pretty_print(pos_b2, std::cout, "pos bd2");
    myUtils::pretty_print(joint_pos1, std::cout, "pos joint");
    std::cout<<"======================================================"<<std::endl;

}

void _SetJointActuatorType(dart::dynamics::SkeletonPtr robot, int actuator_type){
    dart::dynamics::Joint* passive0 = robot->getJoint("joint3");
    dart::dynamics::Joint* passive1 = robot->getJoint("joint4");
    dart::dynamics::Joint* passive2 = robot->getJoint("joint7");
    dart::dynamics::Joint* passive3 = robot->getJoint("joint8");

    passive0->setActuatorType(dart::dynamics::Joint::PASSIVE);
    passive1->setActuatorType(dart::dynamics::Joint::PASSIVE);
    passive2->setActuatorType(dart::dynamics::Joint::PASSIVE);
    passive3->setActuatorType(dart::dynamics::Joint::PASSIVE);

    dart::dynamics::Joint* active1 = robot->getJoint("joint1");
    dart::dynamics::Joint* active2 = robot->getJoint("joint2");
    dart::dynamics::Joint* active3 = robot->getJoint("joint5");
    dart::dynamics::Joint* active4 = robot->getJoint("joint6");
    dart::dynamics::Joint* active5 = robot->getJoint("joint9");
    dart::dynamics::Joint* active6 = robot->getJoint("joint10");
    dart::dynamics::Joint* active7 = robot->getJoint("joint11");


    std::cout<<"======================================================"<<std::endl;
    if( actuator_type == 0 ){
        active1->setActuatorType(dart::dynamics::Joint::SERVO);
        active2->setActuatorType(dart::dynamics::Joint::SERVO);
        active3->setActuatorType(dart::dynamics::Joint::SERVO);
        active4->setActuatorType(dart::dynamics::Joint::SERVO);
        active5->setActuatorType(dart::dynamics::Joint::SERVO);
        active6->setActuatorType(dart::dynamics::Joint::SERVO);
        active7->setActuatorType(dart::dynamics::Joint::SERVO);

        std::cout<<"***** joint types are initialized: Servo mode. "<<std::endl;
    }
    else{
        active1->setActuatorType(dart::dynamics::Joint::FORCE);
        active2->setActuatorType(dart::dynamics::Joint::FORCE);
        active3->setActuatorType(dart::dynamics::Joint::FORCE);
        active4->setActuatorType(dart::dynamics::Joint::FORCE);
        active5->setActuatorType(dart::dynamics::Joint::FORCE);
        active6->setActuatorType(dart::dynamics::Joint::FORCE);
        active7->setActuatorType(dart::dynamics::Joint::FORCE);

        std::cout<<"***** joint types are initialized: Force mode. "<<std::endl;
    }
    std::cout<<"======================================================"<<std::endl;

}

int main(int argc, char** argv) {
    // ========================
    // Parse Yaml for Simulator
    // ========================
    bool isRecord;
    bool b_display_joint_frame;
    bool b_display_target_frame;
    bool b_show;
    int num_steps_per_cycle;
    double servo_rate;
    int actuator_type;
    Eigen::VectorXd q_init(5);
    q_init.setZero();

    try {
        YAML::Node simulation_cfg =
            YAML::LoadFile(THIS_COM "Config/Scorpio/SIMULATION.yaml");
        myUtils::readParameter(simulation_cfg, "is_record", isRecord);
        myUtils::readParameter(simulation_cfg, "display_joint_frame",
                               b_display_joint_frame);
        myUtils::readParameter(simulation_cfg, "display_target_frame",
                               b_display_target_frame);
        myUtils::readParameter(simulation_cfg, "num_steps_per_cycle",
                               num_steps_per_cycle);
        myUtils::readParameter(simulation_cfg, "servo_rate", servo_rate);
        myUtils::readParameter(simulation_cfg, "show_viewer", b_show);
        myUtils::readParameter(simulation_cfg, "actuator_type", actuator_type);
        myUtils::readParameter(simulation_cfg, "q_init", q_init);
        q_init = q_init/180.0*M_PI;

    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
    }

    // ================================
    // Generate world and add skeletons
    // ================================
    dart::simulation::WorldPtr world(new dart::simulation::World);
    dart::utils::DartLoader urdfLoader;
    dart::dynamics::SkeletonPtr ground = urdfLoader.parseSkeleton(
        THIS_COM "RobotModel/Ground/ground_terrain.urdf");
    dart::dynamics::SkeletonPtr scorpio = urdfLoader.parseSkeleton(
        THIS_COM "RobotModel/Robot/Scorpio/Scorpio_Kin.urdf");
    dart::dynamics::SkeletonPtr draco = urdfLoader.parseSkeleton(
        THIS_COM "RobotModel/Robot/Draco/DracoSim_Dart.urdf");
    dart::dynamics::SkeletonPtr table = urdfLoader.parseSkeleton(
         THIS_COM "RobotModel/Environment/Table/table.urdf");
    dart::dynamics::SkeletonPtr box = urdfLoader.parseSkeleton(
         THIS_COM "RobotModel/Environment/Box/box.urdf");

    world->addSkeleton(ground);
    world->addSkeleton(scorpio);
    world->addSkeleton(draco);
    world->addSkeleton(table);
    world->addSkeleton(box);


    // ==================================
    // Friction & Restitution Coefficient
    // ==================================
    double friction(10.);
    double restit(0.0);
    ground->getBodyNode("ground_link")->setFrictionCoeff(friction);
    draco->getBodyNode("Torso")->setFrictionCoeff(friction);
    ground->getBodyNode("ground_link")->setRestitutionCoeff(restit);
    draco->getBodyNode("Torso")->setRestitutionCoeff(restit);
    draco->getBodyNode("rAnkle")->setFrictionCoeff(friction);
    draco->getBodyNode("lAnkle")->setFrictionCoeff(friction);
    draco->getBodyNode("lAnkle")->setRestitutionCoeff(restit);
    draco->getBodyNode("rAnkle")->setRestitutionCoeff(restit);

    Eigen::Vector3d gravity(0.0, 0.0, -9.81);
    world->setGravity(gravity);
    world->setTimeStep(servo_rate);

    // ====================
    // Display Joints Frame
    // ====================
    if (b_display_joint_frame) displayJointFrames(world, scorpio);
    // ====================
    // Display Target Frame
    // ====================
    if (b_display_target_frame) addTargetFrame(world);

    // =====================
    // Initial configuration
    // =====================
    _setInitialConfiguration(scorpio, q_init);
    _setInitialConfiguration(draco);
    _setJointLimitConstraint(draco);
    _setInitialConfiguration_2(box);

    // =====================
    // Robot Mesh Color from URDF
    // =====================
    _SetMeshColorURDF(scorpio);

    // =====================
    // Constraint for Closed-Loop
    // =====================
    _SetJointConstraint(world, scorpio);

    // ================
    // Set passive joint
    // ================
    _SetJointActuatorType(scorpio,actuator_type);

    // ================
    // Print Model Info
    // ================
    // _printRobotModel(robot);

    // ================
    // Wrap a worldnode
    // ================
    osg::ref_ptr<ScorpioWorldNode> node;
    node = new ScorpioWorldNode(world, new DracoInterface(), new ScorpioInterface());

    // Reachability node
    // osg::ref_ptr<ScorpioWorldNodeReach> node;
    // node = new ScorpioWorldNodeReach(world);

    node->setNumStepsPerCycle(num_steps_per_cycle);

    // =====================
    // Create and Set Viewer
    // =====================

    if (b_show) {
        dart::gui::osg::Viewer viewer;
        viewer.addWorldNode(node);
        viewer.simulate(false);
        viewer.switchHeadlights(false);
        ::osg::Vec3 p1(1.0, 0.2, 1.0);
        p1 = p1 * 0.5;
        viewer.getLightSource(0)->getLight()->setPosition(
            ::osg::Vec4(p1[0], p1[1], p1[2], 0.0));
        viewer.getCamera()->setClearColor(osg::Vec4(0.93f, 0.95f, 1.0f, 0.95f));
        viewer.getCamera()->setClearMask(GL_COLOR_BUFFER_BIT |
                                         GL_DEPTH_BUFFER_BIT);
        viewer.addEventHandler(new OneStepProgress(node));

        if (isRecord) {
            std::cout << "[Video Record Enable]" << std::endl;
            viewer.record(THIS_COM "/ExperimentVideo");
        }

        //viewer.setUpViewInWindow(0, 0, 2880, 1800);
        viewer.setUpViewInWindow(1440, 0, 500, 500);
        viewer.getCameraManipulator()->setHomePosition(
            ::osg::Vec3(6.14, 2.28, 3.0) * 1.5, ::osg::Vec3(1.0, 0.2, 0.5),
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
