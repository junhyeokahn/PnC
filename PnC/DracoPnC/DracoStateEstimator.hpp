#pragma once

#include <Eigen/Dense>

#include <Configuration.h>

class DracoStateProvider;
class RobotSystem;
class BasicAccumulation;
class DracoSensorData;
class filter;
class BodyEstimator;
class MomentumEstimator;

class DracoStateEstimator {
   public:
    DracoStateEstimator(RobotSystem* robot);
    ~DracoStateEstimator();

    void initialization(DracoSensorData*);
    void update(DracoSensorData*);

   protected:
    double initial_height_;
    int fixed_foot_;
    Eigen::Vector3d foot_pos_;
    DracoStateProvider* sp_;
    RobotSystem* robot_;

    Eigen::VectorXd curr_config_;
    Eigen::VectorXd curr_qdot_;
    Eigen::Vector3d global_body_euler_zyx_;
    Eigen::Quaternion<double> global_body_quat_;
    Eigen::Vector3d global_body_euler_zyx_dot_;

    BasicAccumulation* ori_est_;
    BodyEstimator* body_est_;
    MomentumEstimator* momentum_est_;
    filter* mocap_x_vel_est_;
    filter* mocap_y_vel_est_;
    filter* x_vel_est_;
    filter* y_vel_est_;

    void _JointUpdate(DracoSensorData* data);
    void _ConfigurationAndModelUpdate();
    void _FootContactUpdate(DracoSensorData* data);
};
