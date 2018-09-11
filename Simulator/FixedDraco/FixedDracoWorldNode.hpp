#pragma once

#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <osgShadow/LightSpacePerspectiveShadowMap>
#include <Eigen/Dense>

class Interface;
class FixedDracoSensorData;
class FixedDracoCommand;

class FixedDracoWorldNode : public dart::gui::osg::WorldNode
{
private:
    Interface* mInterface;
    FixedDracoSensorData* mSensorData;
    FixedDracoCommand* mCommand;

    dart::dynamics::SkeletonPtr mSkel;
    Eigen::VectorXd mTorqueCommand;
    int mDof;

public:
    FixedDracoWorldNode(const dart::simulation::WorldPtr & world,
                        osgShadow::MinimalShadowMap *);
    virtual ~FixedDracoWorldNode();

    void customPreStep() override;
};
