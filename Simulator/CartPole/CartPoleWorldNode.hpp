#pragma once

#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <osgShadow/LightSpacePerspectiveShadowMap>
#include <Eigen/Dense>

class Interface;
class CartPoleSensorData;

class CartPoleWorldNode : public dart::gui::osg::WorldNode
{
private:
    Interface* mInterface;
    CartPoleSensorData* mSensorData;

    dart::dynamics::SkeletonPtr mSkel;
    Eigen::VectorXd mTorqueCommand;
    int mDof;

    Eigen::VectorXd mInit;

public:
    CartPoleWorldNode(const dart::simulation::WorldPtr & world,
                      osgShadow::MinimalShadowMap *);
    virtual ~CartPoleWorldNode();

    void customPreStep() override;
};
