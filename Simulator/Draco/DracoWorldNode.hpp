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
    void _check_foot_contact( bool & rfoot_contact, bool & lfoot_contact );
    void _check_collision();
    void _hold_xy();

    dart::dynamics::SkeletonPtr mSkel;
    dart::dynamics::SkeletonPtr mGround;
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
    Eigen::VectorXd q_sim_;

    dart::simulation::WorldPtr world_;
};
