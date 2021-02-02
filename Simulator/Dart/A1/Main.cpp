#include <Configuration.h>
#include <Simulator/Dart/A1/A1WorldNode.hpp>
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
  OneStepProgress(A1WorldNode* worldnode) : worldnode_(worldnode) {}

  /** Deprecated, Handle events, return true if handled, false otherwise. */
  virtual bool handle(const osgGA::GUIEventAdapter& ea,
                      osgGA::GUIActionAdapter& /*aa*/) {
    if (ea.getEventType() == osgGA::GUIEventAdapter::KEYUP) {
      // custom buttons
      // World Node Buttons
      if (ea.getKey() == 'p') {
        worldnode_->enableButtonPFlag();
      }
      if (ea.getKey() == 'r') {
        worldnode_->enableButtonRFlag();
      }
      if (ea.getKey() == 'w') {
        worldnode_->enableButtonWFlag();
      }
      if (ea.getKey() == 'a') {
        worldnode_->enableButtonAFlag();
      }
      if (ea.getKey() == 's') {
        worldnode_->enableButtonSFlag();
      }
      if (ea.getKey() == 'd') {
        worldnode_->enableButtonDFlag();
      }
      if (ea.getKey() == 'q') {
        worldnode_->enableButtonQFlag();
      }
      if (ea.getKey() == 'e') {
        worldnode_->enableButtonEFlag();
      }
      if (ea.getKey() == 'x') {
        worldnode_->enableButtonXFlag();
      }
      if (ea.getKey() == 'j') {
        worldnode_->enableButtonJFlag();
      }
      if (ea.getKey() == 'k') {
        worldnode_->enableButtonKFlag();
      }
      if (ea.getKey() == 'h') {
        worldnode_->enableButtonHFlag();
      }
      if (ea.getKey() == 'l') {
        worldnode_->enableButtonLFlag();
      }
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
  A1WorldNode* worldnode_;
};

void _setJointLimitConstraint(dart::dynamics::SkeletonPtr robot) {
  for (int i = 0; i < robot->getNumJoints(); ++i) {
    dart::dynamics::Joint* joint = robot->getJoint(i);
    // std::cout << i << "th" << std::endl;
    // std::cout << joint->isPositionLimitEnforced() << std::endl;
    joint->setPositionLimitEnforced(true);
  }
}

void setDampingCoef(dart::dynamics::SkeletonPtr robot) {
  for (int i = 6; i < robot->getNumDofs(); ++i) {
    dart::dynamics::DegreeOfFreedom* dof = robot->getDof(i);
    std::cout << i << "th" << std::endl;
    std::cout << dof->getName() << std::endl;
    std::cout << "child body node name : " << dof->getChildBodyNode()->getName()
              << std::endl;
    std::cout << dof->getCoulombFriction() << std::endl;
    std::cout << dof->getDampingCoefficient() << std::endl;
  }
}

void _printRobotModel(dart::dynamics::SkeletonPtr robot) {
  // for (int i = 0; i < robot->getNumBodyNodes(); ++i) {
  //   dart::dynamics::BodyNodePtr bn = robot->getBodyNode(i);
  //   std::cout << i << "th" << std::endl;
  //   std::cout << bn->getName() << std::endl;
  //   std::cout << bn->getMass() << std::endl;
  // }

  for (int i = 0; i < robot->getNumJoints(); ++i) {
    dart::dynamics::Joint* joint = robot->getJoint(i);
    std::cout << i << "th" << std::endl;
    std::cout << joint->getNumDofs() << std::endl;
  }

  for (int i = 0; i < robot->getNumDofs(); ++i) {
    dart::dynamics::DegreeOfFreedom* dof = robot->getDof(i);
    std::cout << i << "th" << std::endl;
    std::cout << dof->getName() << std::endl;
    std::cout << "child body node name : " <<
    dof->getChildBodyNode()->getName() << std::endl; std::cout <<
    dof->getCoulombFriction() << std::endl;
  }

  std::cout << "num dof" << std::endl;
  std::cout << robot->getNumDofs() << std::endl;
  std::cout << robot->getNumJoints() << std::endl;
  // std::cout << "mass mat row" << std::endl;
  // std::cout << robot->getMassMatrix().rows() << std::endl;
  // std::cout << robot->getMassMatrix().cols() << std::endl;
  // std::cout << "q" << std::endl;
  // std::cout << robot->getPositions() << std::endl;
  // std::cout << "robot total mass" << std::endl;
  // std::cout << robot->getMass() << std::endl;
  std::cout << "robot position" << std::endl;
  std::cout << robot->getPositions() << std::endl;

  exit(0);
}

void _setInitialConfiguration(dart::dynamics::SkeletonPtr robot) {
  int frHipIdx = robot->getDof("FR_hip_joint")->getIndexInSkeleton();
  int frThighIdx = robot->getDof("FR_thigh_joint")->getIndexInSkeleton();
  int frKneeIdx = robot->getDof("FR_calf_joint")->getIndexInSkeleton();
  int flHipIdx = robot->getDof("FL_hip_joint")->getIndexInSkeleton();
  int flThighIdx = robot->getDof("FL_thigh_joint")->getIndexInSkeleton();
  int flKneeIdx = robot->getDof("FL_calf_joint")->getIndexInSkeleton();
  int rrHipIdx = robot->getDof("RR_hip_joint")->getIndexInSkeleton();
  int rrThighIdx = robot->getDof("RR_thigh_joint")->getIndexInSkeleton();
  int rrKneeIdx = robot->getDof("RR_calf_joint")->getIndexInSkeleton();
  int rlHipIdx = robot->getDof("RL_hip_joint")->getIndexInSkeleton();
  int rlThighIdx = robot->getDof("RL_thigh_joint")->getIndexInSkeleton();
  int rlKneeIdx = robot->getDof("RL_calf_joint")->getIndexInSkeleton();

  int initPos(1);  // 0 : Home, 1 : Simulation, 2 : Experiment
  Eigen::VectorXd q = robot->getPositions();

  switch (initPos) {
    case 0: {
      q[2] = 0.35;//Torso Height

      q[frHipIdx] = 0.;
      q[flHipIdx] = 0.;
      q[rrHipIdx] = 0.;
      q[rlHipIdx] = 0.;

      q[frThighIdx] = M_PI/4.;
      q[flThighIdx] = M_PI/4.;
      q[rrThighIdx] = M_PI/4.;
      q[rlThighIdx] = M_PI/4.;

      q[frKneeIdx] = -M_PI/2.;
      q[flKneeIdx] = -M_PI/2.;
      q[rrKneeIdx] = -M_PI/2.;
      q[rlKneeIdx] = -M_PI/2.;
 
      break;
    }
    case 1: {
      q[2] = 0.3;//Torso Height

      q[frHipIdx] = 0.;
      q[flHipIdx] = 0.;
      q[rrHipIdx] = 0.;
      q[rlHipIdx] = 0.;

      q[frThighIdx] = M_PI/4.;
      q[flThighIdx] = M_PI/4.;
      q[rrThighIdx] = M_PI/4.;
      q[rlThighIdx] = M_PI/4.;

      q[frKneeIdx] = -M_PI/2.;
      q[flKneeIdx] = -M_PI/2.;
      q[rrKneeIdx] = -M_PI/2.;
      q[rlKneeIdx] = -M_PI/2.;

      break;
    }
    case 2: {
      /*YAML::Node simulation_cfg =
          YAML::LoadFile(THIS_COM "Config/Draco/SIMULATION.yaml");
      double hanging_height(0.0);
      myUtils::readParameter(simulation_cfg, "hanging_height", hanging_height);
      Eigen::VectorXd init_config;
      myUtils::readParameter(simulation_cfg, "initial_configuration",
                             init_config);
      q[2] = hanging_height;
      q.tail(10) = init_config;*/
      break;
    }
    default:
      std::cout << "[wrong initial pos case] in A1/Main.cpp" << std::endl;
  }

  robot->setPositions(q);
}

int main(int argc, char** argv) {
  // =========================================================================
  // Parse Yaml for Simulator
  // =========================================================================
  bool isRecord;
  bool b_display_joint_frame;
  bool b_joint_limit_enforced;
  int num_steps_per_cycle;
  double servo_rate;

  try {
    YAML::Node simulation_cfg =
        YAML::LoadFile(THIS_COM "Config/A1/SIMULATION.yaml");
    myUtils::readParameter(simulation_cfg, "is_record", isRecord);
    myUtils::readParameter(simulation_cfg, "display_joint_frame",
                           b_display_joint_frame);
    myUtils::readParameter(simulation_cfg, "joint_limit_enforced",
                           b_joint_limit_enforced);
    myUtils::readParameter(simulation_cfg, "num_steps_per_cycle",
                           num_steps_per_cycle);
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
      THIS_COM "RobotModel/Robot/A1/a1_sim.urdf");
  world->addSkeleton(ground);
  world->addSkeleton(robot);

  // =========================================================================
  // Friction & Restitution Coefficient (for collision objects)
  // =========================================================================
  double friction(10.);
  double restit(0.0);
  ground->getBodyNode("ground_link")->setFrictionCoeff(friction);
  robot->getBodyNode("trunk")->setFrictionCoeff(friction);
  ground->getBodyNode("ground_link")->setRestitutionCoeff(restit);
  robot->getBodyNode("trunk")->setRestitutionCoeff(restit);

  robot->getBodyNode("FR_hip")->setRestitutionCoeff(restit);
  robot->getBodyNode("FL_hip")->setRestitutionCoeff(restit);
  robot->getBodyNode("RR_hip")->setRestitutionCoeff(restit);
  robot->getBodyNode("RL_hip")->setRestitutionCoeff(restit);
  robot->getBodyNode("FR_hip")->setFrictionCoeff(friction);
  robot->getBodyNode("FL_hip")->setFrictionCoeff(friction);
  robot->getBodyNode("RR_hip")->setFrictionCoeff(friction);
  robot->getBodyNode("RL_hip")->setFrictionCoeff(friction);

  robot->getBodyNode("FR_thigh_shoulder")->setRestitutionCoeff(restit);
  robot->getBodyNode("FL_thigh_shoulder")->setRestitutionCoeff(restit);
  robot->getBodyNode("RR_thigh_shoulder")->setRestitutionCoeff(restit);
  robot->getBodyNode("RL_thigh_shoulder")->setRestitutionCoeff(restit);
  robot->getBodyNode("FR_thigh_shoulder")->setFrictionCoeff(friction);
  robot->getBodyNode("FL_thigh_shoulder")->setFrictionCoeff(friction);
  robot->getBodyNode("RR_thigh_shoulder")->setFrictionCoeff(friction);
  robot->getBodyNode("RL_thigh_shoulder")->setFrictionCoeff(friction);
  
  robot->getBodyNode("FR_thigh")->setRestitutionCoeff(restit);
  robot->getBodyNode("FL_thigh")->setRestitutionCoeff(restit);
  robot->getBodyNode("RR_thigh")->setRestitutionCoeff(restit);
  robot->getBodyNode("RL_thigh")->setRestitutionCoeff(restit);
  robot->getBodyNode("FR_thigh")->setFrictionCoeff(friction);
  robot->getBodyNode("FL_thigh")->setFrictionCoeff(friction);
  robot->getBodyNode("RR_thigh")->setFrictionCoeff(friction);
  robot->getBodyNode("RL_thigh")->setFrictionCoeff(friction);

  robot->getBodyNode("FR_calf")->setRestitutionCoeff(restit);
  robot->getBodyNode("FL_calf")->setRestitutionCoeff(restit);
  robot->getBodyNode("RR_calf")->setRestitutionCoeff(restit);
  robot->getBodyNode("RL_calf")->setRestitutionCoeff(restit);
  robot->getBodyNode("FR_calf")->setFrictionCoeff(friction);
  robot->getBodyNode("FL_calf")->setFrictionCoeff(friction);
  robot->getBodyNode("RR_calf")->setFrictionCoeff(friction);
  robot->getBodyNode("RL_calf")->setFrictionCoeff(friction);
 

  robot->getBodyNode("FR_foot")->setRestitutionCoeff(restit);
  robot->getBodyNode("FL_foot")->setRestitutionCoeff(restit);
  robot->getBodyNode("RR_foot")->setRestitutionCoeff(restit);
  robot->getBodyNode("RL_foot")->setRestitutionCoeff(restit);
  robot->getBodyNode("FR_foot")->setFrictionCoeff(friction);
  robot->getBodyNode("FL_foot")->setFrictionCoeff(friction);
  robot->getBodyNode("RR_foot")->setFrictionCoeff(friction);
  robot->getBodyNode("RL_foot")->setFrictionCoeff(friction);

  Eigen::Vector3d gravity(0.0, 0.0, -9.81);
  world->setGravity(gravity);
  world->setTimeStep(servo_rate);


  // =========================================================================
  // Set Damping
  // =========================================================================
  // setDampingCoef(robot);

  // =========================================================================
  // Display Joints Frame
  // =========================================================================
  if (b_display_joint_frame) displayJointFrames(world, robot);
  // =========================================================================
  // Initial configuration
  // =========================================================================
  _setInitialConfiguration(robot);
  // =========================================================================
  // Enabel Joit Limits
  // =========================================================================
  if (b_joint_limit_enforced) _setJointLimitConstraint(robot);


  // =========================================================================
  // Print Model Info
  // =========================================================================
  // _printRobotModel(robot);

  float minLightMargin = 10.f;
  float maxFarPlane = 0;
  unsigned int texSize = 1024;
  unsigned int baseTexUnit = 0;
  unsigned int shadowTexUnit = 1;

  // =========================================================================
  // Wrap a worldnode
  // =========================================================================
  osg::ref_ptr<A1WorldNode> node;
  node = new A1WorldNode(world); // The error is in A1WorldNode
  node->setNumStepsPerCycle(num_steps_per_cycle);

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
      ::osg::Vec3(5.14, 2.28, 3.0) * 0.8, ::osg::Vec3(0.0, 0.2, 0.5),
      ::osg::Vec3(0.0, 0.0, 1.0));
  viewer.setCameraManipulator(viewer.getCameraManipulator());
  viewer.run();
}
