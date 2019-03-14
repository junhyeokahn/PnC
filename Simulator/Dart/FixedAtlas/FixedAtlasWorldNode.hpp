#pragma once

#include <Eigen/Dense>
#include <dart/dart.hpp>
#include <dart/gui/GLFuncs.hpp>
#include <dart/gui/osg/osg.hpp>
#include <osgShadow/LightSpacePerspectiveShadowMap>

class EnvInterface;
class FixedAtlasSensorData;
class FixedAtlasCommand;

class FixedAtlasWorldNode : public dart::gui::osg::WorldNode {
   private:
    EnvInterface* mInterface;
    FixedAtlasSensorData* mSensorData;
    FixedAtlasCommand* mCommand;

    dart::dynamics::SkeletonPtr mSkel;
    dart::dynamics::SkeletonPtr mGround;
    Eigen::VectorXd mTorqueCommand;

    double count_;
    double t_;
    double servo_rate_;

   public:
    FixedAtlasWorldNode(const dart::simulation::WorldPtr& world,
                        osgShadow::MinimalShadowMap*);

    virtual ~FixedAtlasWorldNode(); ////////====== why virtual function????????

    void customPreStep() override;

    dart::simulation::WorldPtr world_;
};
