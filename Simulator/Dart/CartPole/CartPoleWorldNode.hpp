#pragma once

#include <Eigen/Dense>
#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <osgShadow/LightSpacePerspectiveShadowMap>

#include <Utils/General/Clock.hpp>

class EnvInterface;
class CartPoleSensorData;
class CartPoleCommand;

class CartPoleWorldNode : public dart::gui::osg::WorldNode {
   private:
    EnvInterface* interface_;
    CartPoleSensorData* sensor_data_;
    CartPoleCommand* cmd_;

    dart::dynamics::SkeletonPtr cart_pole_;
    Eigen::VectorXd torque_cmd_;

   public:
    CartPoleWorldNode(const dart::simulation::WorldPtr& world,
                      osgShadow::MinimalShadowMap*);
    CartPoleWorldNode(const dart::simulation::WorldPtr& world,
                      osgShadow::MinimalShadowMap*, int mpi_idx, int env_idx);
    virtual ~CartPoleWorldNode();

    void customPreStep() override;

    dart::simulation::WorldPtr world_;
    void ResetSim_();

    Clock clock_;
};
