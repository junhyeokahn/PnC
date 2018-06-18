#pragma once

#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <osgShadow/LightSpacePerspectiveShadowMap>
#include <Eigen/Dense>

class Interface;
class PendulumSensorData;

class PendulumWorldNode : public dart::gui::osg::WorldNode
{
private:
    Interface* mInterface;
    PendulumSensorData* mSensorData;

    dart::dynamics::SkeletonPtr mSkel;
    Eigen::VectorXd mTorqueCommand;
    int mDof;

    Eigen::VectorXd mInit;

public:
    PendulumWorldNode(const dart::simulation::WorldPtr & world,
                      osgShadow::MinimalShadowMap *);
    virtual ~PendulumWorldNode();

    void customPreStep() override;
};
