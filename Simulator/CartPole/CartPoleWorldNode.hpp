#pragma once

#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <osgShadow/LightSpacePerspectiveShadowMap>
#include <Eigen/Dense>

#include "Utils/Clock.hpp"

class Interface;
class CartPoleSensorData;
class CartPoleCommand;

class CartPoleWorldNode : public dart::gui::osg::WorldNode
{
private:
    Interface* interface_;
    CartPoleSensorData* sensor_data_;
    CartPoleCommand* cmd_;

    dart::dynamics::SkeletonPtr cart_pole_;
    Eigen::VectorXd torque_cmd_;

public:
    CartPoleWorldNode(const dart::simulation::WorldPtr & world,
                   osgShadow::MinimalShadowMap *);
    virtual ~CartPoleWorldNode();

    void customPreStep() override;

    dart::simulation::WorldPtr world_;

    Clock clock_;
};
