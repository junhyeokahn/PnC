#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/utils.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include "CartPoleWorldNode.hpp"
#include "Configuration.h"

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
                int numStepProgress(10);
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

    for (int i = 0; i < robot->getNumBodyNodes(); ++i) {
        dart::dynamics::BodyNodePtr bn = robot->getBodyNode(i);
        std::cout << i << "th" << std::endl;
        std::cout << bn->getName() << std::endl;
    }

    for (int i = 0; i < robot->getNumJoints(); ++i) {
        dart::dynamics::Joint* joint = robot->getJoint(i);
        std::cout << i << "th" << std::endl;
        std::cout << joint->getNumDofs() << std::endl;
    }

    for (int i = 0; i < robot->getNumDofs(); ++i) {
        dart::dynamics::DegreeOfFreedom* dof = robot->getDof(i);
        std::cout << i << "th" << std::endl;
        std::cout << dof->getName() << std::endl;
        std::cout << "child body node name : " << dof->getChildBodyNode()->getName() << std::endl;
    }

    std::cout << robot->getNumDofs() << std::endl;
    std::cout << robot->getNumJoints() << std::endl;
    std::cout << robot->getMassMatrix().rows() << std::endl;
    std::cout << robot->getMassMatrix().cols() << std::endl;
    exit(0);
}

void _setInitialConfiguration(dart::dynamics::SkeletonPtr robot) {

    Eigen::VectorXd q(robot->getNumDofs());
    q.setZero();

    q[1] = 1.0;

    robot->setPositions(q);
}

int main() {
    // ================================
    // Generate world and add skeletons
    // ================================
    dart::simulation::WorldPtr world(new dart::simulation::World);
    dart::utils::DartLoader urdfLoader;
    dart::dynamics::SkeletonPtr robot = urdfLoader.parseSkeleton(
            THIS_COM"Simulator/SimulationModel/RobotModel/CartPole/CartPole.urdf");
    world->addSkeleton(robot);
    Eigen::Vector3d gravity(0.0, 0.0, -9.81);
    world->setGravity(gravity);
    world->setTimeStep(1.0/2000);

    // ====================
    // Display Joints Frame
    // ====================
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
    node->setNumStepsPerCycle(30);

    // =====================
    // Create and Set Viewer
    // =====================

    dart::gui::osg::Viewer viewer;
    viewer.addWorldNode(node);
    viewer.simulate(false);
    viewer.switchHeadlights(false);
    msm->setLight(viewer.getLightSource(0)->getLight());
    ::osg::Vec3 p1(1.0, 0.2 , 1.5);
    viewer.getLightSource(0)->getLight()->setPosition(::osg::Vec4(p1[0], p1[1], p1[2], 0.0));
    viewer.getCamera()->setClearColor(osg::Vec4(0.93f, 0.95f, 1.0f, 0.95f));
    viewer.getCamera()->setClearMask(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    viewer.addEventHandler(new OneStepProgress(node) );
    //viewer.record(THIS_COM"/ExperimentVideo");

    //std::cout << "=====================================" << std::endl;
    //std::cout << viewer.getInstructions() << std::endl;
    viewer.setUpViewInWindow(0, 0, 2880, 1800);
    //viewer.setUpViewInWindow(0, 0, 500, 500);
    viewer.getCameraManipulator()->setHomePosition(
            //::osg::Vec3( 5.14,  3.28, 1.8)*0.5,
            ::osg::Vec3( 0.0,  -1.5, 0.9)*1.3,
            ::osg::Vec3( 0.0,  0.0, 0.3),
            ::osg::Vec3(0.0, 0.0, 1.0));
    viewer.setCameraManipulator(viewer.getCameraManipulator());
    viewer.run();
}
