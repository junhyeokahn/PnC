#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/utils.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include "Utils/IO/IOUtilities.hpp"
#include "CartPoleWorldNode.hpp"
#include "Configuration.h"
#include <random>

void displayJointFrames(
    const dart::simulation::WorldPtr& world, 
    const dart::dynamics::SkeletonPtr& robot)
{
  for(std::size_t i=0; i < robot->getNumBodyNodes(); ++i)
  {
      dart::dynamics::BodyNode* bn = robot->getBodyNode(i);
      for(std::size_t j=0; j < bn->getNumChildJoints(); ++j)
      {
          const dart::dynamics::Joint* joint = bn->getChildJoint(j);
          const Eigen::Isometry3d offset = joint->getTransformFromParentBodyNode();

          dart::gui::osg::InteractiveFramePtr frame = std::make_shared<dart::gui::osg::InteractiveFrame>
              (bn,joint->getName()+"/frame", offset);

          for(const auto type : {dart::gui::osg::InteractiveTool::ANGULAR, dart::gui::osg::InteractiveTool::PLANAR})
              for(std::size_t i=0; i < 3; ++i)
                      frame->getTool(type, i)->setEnabled(false);

          world->addSimpleFrame(frame);
      }
  }
}

class OneStepProgress : public osgGA::GUIEventHandler
{
public:
    OneStepProgress(CartPoleWorldNode* worldnode): worldnode_(worldnode){  }

    /** Deprecated, Handle events, return true if handled, false otherwise. */
    virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& /*aa*/)
    {
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
    CartPoleWorldNode* worldnode_;
};

void _printRobotModel(dart::dynamics::SkeletonPtr robot) {

    //for (int i = 0; i < robot->getNumBodyNodes(); ++i) {
        //dart::dynamics::BodyNodePtr bn = robot->getBodyNode(i);
        //std::cout << i << "th" << std::endl;
        //std::cout << bn->getName() << std::endl;
        //std::cout << bn->getMass() << std::endl;
    //}

    //for (int i = 0; i < robot->getNumJoints(); ++i) {
        //dart::dynamics::Joint* joint = robot->getJoint(i);
        //std::cout << i << "th" << std::endl;
        //std::cout << joint->getNumDofs() << std::endl;
    //}

    //for (int i = 0; i < robot->getNumDofs(); ++i) {
        //dart::dynamics::DegreeOfFreedom* dof = robot->getDof(i);
        //std::cout << i << "th" << std::endl;
        //std::cout << dof->getName() << std::endl;
        //std::cout << "child body node name : " << dof->getChildBodyNode()->getName() << std::endl;
        //std::cout << dof->getCoulombFriction() << std::endl;
    //}

    //std::cout << "num dof" << std::endl;
    //std::cout << robot->getNumDofs() << std::endl;
    //std::cout << robot->getNumJoints() << std::endl;
    //std::cout << "mass mat row" << std::endl;
    //std::cout << robot->getMassMatrix().rows() << std::endl;
    //std::cout << robot->getMassMatrix().cols() << std::endl;
    //std::cout << "q" << std::endl;
    //std::cout << robot->getPositions() << std::endl;
    //std::cout << "robot total mass" << std::endl;
    //std::cout << robot->getMass() << std::endl;
    std::cout << "robot position" << std::endl;
    std::cout << robot->getPositions() << std::endl;

    exit(0);
}

void _setInitialConfiguration(dart::dynamics::SkeletonPtr robot){
    YAML::Node simulation_cfg =
        YAML::LoadFile(THIS_COM"Config/CartPole/SIMULATION.yaml");
    Eigen::VectorXd init_state_lower_bound, init_state_upper_bound;
    myUtils::readParameter(simulation_cfg, "init_state_lower_bound", init_state_lower_bound);
    myUtils::readParameter(simulation_cfg, "init_state_upper_bound", init_state_upper_bound);

    Eigen::VectorXd q = robot->getPositions();
    Eigen::VectorXd qdot = robot->getVelocities();

    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()

    for (int i = 0; i < 2; ++i) {
        std::uniform_real_distribution<> dis(init_state_lower_bound[i], init_state_upper_bound[i]);
        q[i] = dis(gen);
    }
    for (int i = 0; i < 2; ++i) {
        std::uniform_real_distribution<> dis(init_state_lower_bound[i+2], init_state_upper_bound[i+2]);
        qdot[i] = dis(gen);
    }
    robot->setPositions(q);
    robot->setVelocities(qdot);
    //myUtils::pretty_print(q, std::cout, "q");
    //myUtils::pretty_print(qdot, std::cout, "qdot");
}

int main() {
    // ========================
    // Parse Yaml for Simulator
    // ========================
    YAML::Node simulation_cfg =
        YAML::LoadFile(THIS_COM"Config/CartPole/SIMULATION.yaml");
    bool isRecord;
    myUtils::readParameter(simulation_cfg, "is_record", isRecord);
    bool b_display_joint_frame;
    myUtils::readParameter(simulation_cfg, "display_joint_frame", b_display_joint_frame);
    int num_steps_per_cycle;
    myUtils::readParameter(simulation_cfg, "num_steps_per_cycle", num_steps_per_cycle);
    double servo_rate;
    myUtils::readParameter(simulation_cfg, "servo_rate", servo_rate);
    bool b_viewer;
    myUtils::readParameter(simulation_cfg, "show_viewer", b_viewer);

    // ================================
    // Generate world and add skeletons
    // ================================
    dart::simulation::WorldPtr world(new dart::simulation::World);
    dart::utils::DartLoader urdfLoader;
    dart::dynamics::SkeletonPtr robot = urdfLoader.parseSkeleton(
            THIS_COM"RobotSystem/RobotModel/Robot/CartPole/CartPole.urdf");
    world->addSkeleton(robot);

    Eigen::Vector3d gravity(0.0, 0.0, -9.81);
    world->setGravity(gravity);
    world->setTimeStep(servo_rate);

    // ====================
    // Display Joints Frame
    // ====================
    //if (b_display_joint_frame)
        //displayJointFrames(world, robot);

    // =====================
    // Initial configuration
    // =====================
    _setInitialConfiguration(robot);

    // ================
    // Print Model Info
    // ================
    //_printRobotModel(robot);

    osg::ref_ptr<osgShadow::MinimalShadowMap> msm =
        new osgShadow::LightSpacePerspectiveShadowMapDB;

    float minLightMargin = 10.f;
    float maxFarPlane = 0;
    unsigned int texSize = 1024;
    unsigned int baseTexUnit = 0;
    unsigned int shadowTexUnit = 1;

    msm->setMinLightMargin( minLightMargin );
    msm->setMaxFarPlane( maxFarPlane );
    msm->setTextureSize( ::osg::Vec2s( texSize, texSize ) );
    msm->setShadowTextureCoordIndex( shadowTexUnit );
    msm->setShadowTextureUnit( shadowTexUnit );
    msm->setBaseTextureCoordIndex( baseTexUnit );
    msm->setBaseTextureUnit( baseTexUnit );

    // ================
    // Wrap a worldnode
    // ================
    osg::ref_ptr<CartPoleWorldNode> node
        = new CartPoleWorldNode(world, msm);
    node->setNumStepsPerCycle(num_steps_per_cycle);

    // =====================
    // Create and Set Viewer
    // =====================

    if (b_viewer) {
        dart::gui::osg::Viewer viewer;
        viewer.addWorldNode(node);
        viewer.simulate(false);
        viewer.switchHeadlights(false);
        msm->setLight(viewer.getLightSource(0)->getLight());
        ::osg::Vec3 p1(1.0, 0.2, 1.0);
        p1 = p1*0.7;
        viewer.getLightSource(0)->getLight()->setPosition(::osg::Vec4(p1[0], p1[1], p1[2], 0.0));
        viewer.getCamera()->setClearColor(osg::Vec4(0.93f, 0.95f, 1.0f, 0.95f));
        viewer.getCamera()->setClearMask(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
        viewer.addEventHandler(new OneStepProgress(node) );

        if (isRecord) {
            std::cout << "[Video Record Enable]" << std::endl;
            viewer.record(THIS_COM"/ExperimentVideo");
        }

        viewer.setUpViewInWindow(0, 0, 500, 500);
        viewer.getCameraManipulator()->setHomePosition(
                ::osg::Vec3( 0.0,  -1.5, 0.9) * 10,
                ::osg::Vec3( 0.0,  0.0, 0.3),
                ::osg::Vec3(0.0, 0.0, 1.0));
        viewer.setCameraManipulator(viewer.getCameraManipulator());
        viewer.run();
    } else {
        while(true){
            node->customPreStep();
            node->getWorld()->step();
            node->customPostStep();
        }
    }
}
