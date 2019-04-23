#pragma once

#include <Eigen/Dense>
#include <dart/dart.hpp>
#include <dart/gui/GLFuncs.hpp>
#include <dart/gui/osg/osg.hpp>
#include <Configuration.h>
#include <dart/utils/utils.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <osgShadow/LightSpacePerspectiveShadowMap>
#include <unsupported/Eigen/CXX11/Tensor>

class ScorpioWorldNode : public dart::gui::osg::WorldNode {
   private:
    dart::dynamics::SkeletonPtr mSkel_;
    dart::dynamics::SkeletonPtr mGround_;

    dart::dynamics::Joint* active1;
    dart::dynamics::Joint* active2;
    dart::dynamics::Joint* active3;
    dart::dynamics::Joint* active4;
    dart::dynamics::Joint* active5;
    dart::dynamics::Joint* active6;
    dart::dynamics::Joint* active7;

    int mDof_;
    int mActuatingDof_;
    double servo_rate_;
    int count_;
    double t_;

    Eigen::VectorXd Amp_;
    Eigen::VectorXd Freq_;
    Eigen::VectorXd kp_;
    Eigen::VectorXd kv_;
    int actuator_type_;
    int sim_case_;

    Eigen::VectorXd q_init_;
    Eigen::VectorXd q_limit_u_;
    Eigen::VectorXd q_limit_l_;

   public:
    ScorpioWorldNode(const dart::simulation::WorldPtr& world);
    virtual ~ScorpioWorldNode();

    void SetActivePosition(Eigen::VectorXd & pos_des);
    void SetActiveForce(Eigen::VectorXd & des_force);
    void SetActiveVelocity(Eigen::VectorXd & des_vel);
    void GetActiveJointInfo(Eigen::VectorXd & cur_pos, Eigen::VectorXd & cur_vel);

    void UpdateSystem(Eigen::VectorXd & q_activate);
    void customPreStep() override;

    void ComputeWorkspace(double rad_interval);
    void PrepareWorkspaceAnalysis(double dx);
    void UpdateWorkspace(Eigen::Vector3d & pos, Eigen::Tensor<double,3> & Workcount);

    dart::simulation::WorldPtr world_;
    Eigen::VectorXd joint_idx_;
    Eigen::VectorXi num_cart_;
    
    double dq_input_;
    double dx_input_;

    double delta_cart_;
    Eigen::VectorXd upper_cart_;
    Eigen::VectorXd lower_cart_;
};
