#pragma once

#include <Configuration.h>
#include <Eigen/Dense>

class AtlasStateProvider;
class RobotSystem;
class BasicAccumulation;
class AtlasSensorData;

class AtlasStateEstimator {
   public:
    AtlasStateEstimator(RobotSystem* robot);
    ~AtlasStateEstimator();

    void Initialization(AtlasSensorData*);
    void Update(AtlasSensorData*);

   protected:
    double initial_height_;
    int fixed_foot_;
    Eigen::Vector3d foot_pos_;
    AtlasStateProvider* sp_;
    RobotSystem* robot_;

    Eigen::VectorXd curr_config_;
    Eigen::VectorXd curr_qdot_;
    Eigen::Vector3d global_body_euler_zyx_;
    Eigen::Quaternion<double> global_body_quat_;
    Eigen::Vector3d global_body_euler_zyx_dot_;

    BasicAccumulation* ori_est_;

    void _JointUpdate(AtlasSensorData* data);
    void _ConfigurationAndModelUpdate();
    void _FootContactUpdate(AtlasSensorData* data);
};
