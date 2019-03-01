#pragma once

#include <Eigen/Dense>
#include <dart/dart.hpp>
#include <dart/gui/GLFuncs.hpp>
#include <dart/gui/osg/osg.hpp>
#include <osgShadow/LightSpacePerspectiveShadowMap>

class EnvInterface;
class FixedDracoSensorData;
class FixedDracoCommand;
class FixedDracoLedPosAnnouncer;

class FixedDracoWorldNode : public dart::gui::osg::WorldNode {
   private:
    EnvInterface* mInterface;
    FixedDracoSensorData* mSensorData;
    FixedDracoCommand* mCommand;

    dart::dynamics::SkeletonPtr mSkel;
    dart::dynamics::SkeletonPtr mGround;
    Eigen::VectorXd mTorqueCommand;

    double count_;
    double t_;
    double servo_rate_;

   public:
    FixedDracoWorldNode(const dart::simulation::WorldPtr& world,
                        osgShadow::MinimalShadowMap*);
    virtual ~FixedDracoWorldNode();

    void customPreStep() override;

    dart::simulation::WorldPtr world_;
};
