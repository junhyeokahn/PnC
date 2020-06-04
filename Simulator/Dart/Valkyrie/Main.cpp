#include <Configuration.h>
#include <Simulator/Dart/Valkyrie/ValkyrieWorldNode.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>

void display_joint_frames(const dart::simulation::WorldPtr& world,
                          const dart::dynamics::SkeletonPtr& robot) {
  for (std::size_t i = 0; i < robot->getNumBodyNodes(); ++i) {
    dart::dynamics::BodyNode* bn = robot->getBodyNode(i);
    for (std::size_t j = 0; j < bn->getNumChildJoints(); ++j) {
      const dart::dynamics::Joint* joint = bn->getChildJoint(j);
      const Eigen::Isometry3d offset = joint->getTransformFromParentBodyNode();

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

class OneStepProgress : public osgGA::GUIEventHandler {
 public:
  OneStepProgress(ValkyrieWorldNode* worldnode) : worldnode_(worldnode) {}

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
  ValkyrieWorldNode* worldnode_;
};

void set_joint_limit(dart::dynamics::SkeletonPtr robot) {
  for (int i = 0; i < robot->getNumJoints(); ++i) {
    dart::dynamics::Joint* joint = robot->getJoint(i);
    joint->setPositionLimitEnforced(true);
  }
}

void print_robot_model(dart::dynamics::SkeletonPtr robot) {
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
    // std::cout << "child body node name and mass : "
    //<< dof->getChildBodyNode()->getName() << " , "
    //<< dof->getChildBodyNode()->getMass() << std::endl;
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

  // std::cout << "right" << std::endl;
  // std::cout << robot->getBodyNode("rightCOP_Frame")
  //->getWorldTransform()
  //.translation()
  //<< std::endl;
  // std::cout << "left" << std::endl;
  // std::cout << robot->getBodyNode("leftCOP_Frame")
  //->getWorldTransform()
  //.translation()
  //<< std::endl;

  exit(0);
}

void set_init_config(dart::dynamics::SkeletonPtr robot) {
  int leftHipPitch = robot->getDof("leftHipPitch")->getIndexInSkeleton();
  int rightHipPitch = robot->getDof("rightHipPitch")->getIndexInSkeleton();
  int leftKneePitch = robot->getDof("leftKneePitch")->getIndexInSkeleton();
  int rightKneePitch = robot->getDof("rightKneePitch")->getIndexInSkeleton();
  int leftAnklePitch = robot->getDof("leftAnklePitch")->getIndexInSkeleton();
  int rightAnklePitch = robot->getDof("rightAnklePitch")->getIndexInSkeleton();

  int rightShoulderPitch =
      robot->getDof("rightShoulderPitch")->getIndexInSkeleton();
  int rightShoulderRoll =
      robot->getDof("rightShoulderRoll")->getIndexInSkeleton();
  int rightElbowPitch = robot->getDof("rightElbowPitch")->getIndexInSkeleton();
  int rightForearmYaw = robot->getDof("rightForearmYaw")->getIndexInSkeleton();

  int leftShoulderPitch =
      robot->getDof("leftShoulderPitch")->getIndexInSkeleton();
  int leftShoulderRoll =
      robot->getDof("leftShoulderRoll")->getIndexInSkeleton();
  int leftElbowPitch = robot->getDof("leftElbowPitch")->getIndexInSkeleton();
  int leftForearmYaw = robot->getDof("leftForearmYaw")->getIndexInSkeleton();

  int rightHipYaw = robot->getDof("rightHipYaw")->getIndexInSkeleton();

  Eigen::VectorXd q = robot->getPositions();
  q[2] = 2.5 - 1.365 - 0.11;
  q[leftHipPitch] = -0.6;
  q[rightHipPitch] = -0.6;
  q[leftKneePitch] = 1.2;
  q[rightKneePitch] = 1.2;
  q[leftAnklePitch] = -0.6;
  q[rightAnklePitch] = -0.6;

  q[0] = 0.5;
  q[1] = 0.5;
  q[3] = 3.14159 / 6.0;
  // q[rightHipYaw] = -3.14159/12.0;

  q[rightShoulderPitch] = 0.2;
  q[rightShoulderRoll] = 1.1;
  q[rightElbowPitch] = 0.4;
  q[rightForearmYaw] = 1.5;

  q[leftShoulderPitch] = 0.2;
  q[leftShoulderRoll] = -1.1;
  q[leftElbowPitch] = -0.4;  // TODO
  q[leftForearmYaw] = 1.5;

  robot->setPositions(q);
}

int main(int argc, char** argv) {
  double servo_rate;
  bool isRecord;
  bool b_show_joint_frame;
  try {
    YAML::Node simulation_cfg =
        YAML::LoadFile(THIS_COM "Config/Valkyrie/SIMULATION.yaml");
    myUtils::readParameter(simulation_cfg, "servo_rate", servo_rate);
    myUtils::readParameter(simulation_cfg, "is_record", isRecord);
    myUtils::readParameter(simulation_cfg, "show_joint_frame",
                           b_show_joint_frame);
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
      THIS_COM "RobotModel/Robot/Valkyrie/ValkyrieSim_Dart.urdf");

  world->addSkeleton(ground);
  world->addSkeleton(robot);

  // =========================================================================
  // Friction & Restitution Coefficient
  // =========================================================================
  double friction(0.9);
  double restit(0.0);
  ground->getBodyNode("ground_link")->setFrictionCoeff(friction);
  robot->getBodyNode("rightFoot")->setFrictionCoeff(friction);
  robot->getBodyNode("leftFoot")->setFrictionCoeff(friction);

  Eigen::Vector3d gravity(0.0, 0.0, -9.81);
  world->setGravity(gravity);
  world->setTimeStep(servo_rate);

  // =========================================================================
  // Display Joints Frame
  // =========================================================================
  if (b_show_joint_frame) display_joint_frames(world, robot);

  // =========================================================================
  // Initial configuration
  // =========================================================================
  set_init_config(robot);

  // =========================================================================
  // Enabel Joit Limits
  // =========================================================================
  // set_joint_limit(robot);

  // =========================================================================
  // Print Model Info
  // =========================================================================
  // print_robot_model(robot);

  // =========================================================================
  // Wrap a worldnode
  // =========================================================================
  osg::ref_ptr<ValkyrieWorldNode> node;
  node = new ValkyrieWorldNode(world);
  node->setNumStepsPerCycle(30);

  // =========================================================================
  // Create and Set Viewer
  // =========================================================================
  dart::gui::osg::Viewer viewer;
  viewer.addWorldNode(node);
  viewer.simulate(false);
  viewer.switchHeadlights(false);
  ::osg::Vec3 p1(1.0, 0.2, 1.0);
  p1 = p1 * 0.7;
  viewer.getLightSource(0)->getLight()->setPosition(
      ::osg::Vec4(p1[0], p1[1], p1[2], 0.0));
  viewer.getCamera()->setClearColor(osg::Vec4(0.93f, 0.95f, 1.0f, 0.95f));
  viewer.getCamera()->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  viewer.addEventHandler(new OneStepProgress(node));

  if (isRecord) {
    std::cout << "[Video Record Enable]" << std::endl;
    viewer.record(THIS_COM "/ExperimentVideo");
  }

  // viewer.setUpViewInWindow(0, 0, 2880, 1800);
  viewer.setUpViewInWindow(1440, 0, 500, 500);
  viewer.getCameraManipulator()->setHomePosition(
      ::osg::Vec3(5.14, 2.28, 3.0) * 1.5, ::osg::Vec3(0.0, 0.2, 0.5),
      ::osg::Vec3(0.0, 0.0, 1.0));
  viewer.setCameraManipulator(viewer.getCameraManipulator());
  viewer.run();
}
