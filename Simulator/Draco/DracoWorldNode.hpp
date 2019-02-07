#pragma once

#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <osgShadow/LightSpacePerspectiveShadowMap>
#include <Eigen/Dense>

#include "Utils/General/Clock.hpp"

class Interface;
class DracoSensorData;
class DracoCommand;
class DracoLedPosAnnouncer;

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
    void _hold_rot();

    dart::dynamics::SkeletonPtr mSkel;
    dart::dynamics::SkeletonPtr mGround;
    Eigen::VectorXd mTorqueCommand;
    int mDof;
    double mReleaseTime;
    double pulling_back_time_;
    double pulling_back_distance_;

    int count_;
    double t_;
    bool b_check_collision_;
    bool b_print_computation_time;

public:
    DracoWorldNode(const dart::simulation::WorldPtr & world,
                      osgShadow::MinimalShadowMap *);
    virtual ~DracoWorldNode();

    void customPreStep() override;
    Eigen::VectorXd mKp;
    Eigen::VectorXd mKd;
    Eigen::VectorXd q_sim_;

    dart::simulation::WorldPtr world_;

    DracoLedPosAnnouncer* led_pos_announcer_;
    Clock clock_;
};
