#pragma once

#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <osgShadow/LightSpacePerspectiveShadowMap>
#include <Eigen/Dense>

class Interface;
class DracoSensorData;
class DracoCommand;

class DracoWorldNode : public dart::gui::osg::WorldNode
{
private:
    Interface* mInterface;
    DracoSensorData* mSensorData;
    DracoCommand* mCommand;

    void _get_imu_data( Eigen::VectorXd & ang_vel,
                        Eigen::VectorXd & acc);

    dart::dynamics::SkeletonPtr mSkel;
    Eigen::VectorXd mTorqueCommand;
    int mDof;
    double mReleaseTime;

public:
    DracoWorldNode(const dart::simulation::WorldPtr & world,
                      osgShadow::MinimalShadowMap *);
    virtual ~DracoWorldNode();

    void customPreStep() override;
    Eigen::VectorXd mKp;
    Eigen::VectorXd mKd;
};
