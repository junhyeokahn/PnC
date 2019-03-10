#pragma once

#include <Eigen/Dense>
#include <dart/dart.hpp>
#include <dart/gui/GLFuncs.hpp>
#include <dart/gui/osg/osg.hpp>
#include <osgShadow/LightSpacePerspectiveShadowMap>

class EnvInterface;
class AtlasSensorData;
class AtlasCommand;

class AtlasWorldNode : public dart::gui::osg::WorldNode {
   private:
    EnvInterface* mInterface;
    AtlasSensorData* mSensorData;
    AtlasCommand* mCommand;

    dart::dynamics::SkeletonPtr mSkel;
    dart::dynamics::SkeletonPtr mGround;
    Eigen::VectorXd mTorqueCommand;

    double count_;
    double t_;
    double servo_rate_;

   public:
    AtlasWorldNode(const dart::simulation::WorldPtr& world,
                        osgShadow::MinimalShadowMap*);

    virtual ~AtlasWorldNode(); ////////====== why virtual function????????

    void customPreStep() override;

    dart::simulation::WorldPtr world_;
};
