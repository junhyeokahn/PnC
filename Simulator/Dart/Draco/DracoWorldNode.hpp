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

    void _get_imu_data(Eigen::VectorXd& ang_vel, Eigen::VectorXd& acc,
                       Eigen::VectorXd& mag);
    void _check_foot_contact(bool& rfoot_contact, bool& lfoot_contact);
    void _check_collision();
    void _hold_xy();
    void _hold_rot();
    void UpdateLedData_();
    void SetParameters_();

    dart::dynamics::SkeletonPtr mSkel;
    dart::dynamics::SkeletonPtr mGround;
    dart::dynamics::SkeletonPtr mStar;
    dart::dynamics::SkeletonPtr mTorus;
    Eigen::VectorXd mTorqueCommand;
    int mDof;
    double mReleaseTime;

    int count_;
    double waiting_time_;
    double t_;
    bool b_check_collision_;
    bool b_print_computation_time;
    double servo_rate_;

   public:
    DracoWorldNode(const dart::simulation::WorldPtr& world);

    virtual ~DracoWorldNode();

    void customPreStep() override;
    Eigen::VectorXd mKp;
    Eigen::VectorXd mKd;
    Eigen::VectorXd q_sim_;

    dart::simulation::WorldPtr world_;

    DracoLedPosAnnouncer* led_pos_announcer_;
    Clock clock_;
};
