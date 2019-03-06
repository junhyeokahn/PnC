#pragma once

#include <Eigen/Dense>
#include <dart/dart.hpp>
#include <dart/gui/GLFuncs.hpp>
#include <dart/gui/osg/osg.hpp>
#include <osgShadow/LightSpacePerspectiveShadowMap>

#include <Utils/General/Clock.hpp>

class EnvInterface;
class DracoSensorData;
class DracoCommand;
class DracoLedPosAnnouncer;

class DracoWorldNode : public dart::gui::osg::WorldNode {
   private:
    EnvInterface* mInterface;
    DracoSensorData* mSensorData;
    DracoCommand* mCommand;

    void _get_imu_data(Eigen::VectorXd& ang_vel, Eigen::VectorXd& acc);
    void _check_foot_contact(bool& rfoot_contact, bool& lfoot_contact);
    void _check_collision();
    void GetFTSensorData_();
    void _hold_xy();
    void _hold_rot();
    void UpdateLedData_();
    void UpdateTargetLocation_();

    dart::dynamics::SkeletonPtr mSkel;
    dart::dynamics::SkeletonPtr mGround;
    Eigen::VectorXd mTorqueCommand;
    int mDof;
    double mReleaseTime;

    int count_;
    double waiting_time_;
    int mpi_idx_;
    int env_idx_;
    double t_;
    bool b_check_collision_;
    bool b_print_computation_time;
    double servo_rate_;

   public:
    DracoWorldNode(const dart::simulation::WorldPtr& world,
                   osgShadow::MinimalShadowMap*);
    DracoWorldNode(const dart::simulation::WorldPtr& world,
                   osgShadow::MinimalShadowMap*, int mpi_idx, int env_idx);
    virtual ~DracoWorldNode();

    void customPreStep() override;
    Eigen::VectorXd mKp;
    Eigen::VectorXd mKd;
    Eigen::VectorXd q_sim_;

    dart::simulation::WorldPtr world_;

    DracoLedPosAnnouncer* led_pos_announcer_;
    Clock clock_;
};
