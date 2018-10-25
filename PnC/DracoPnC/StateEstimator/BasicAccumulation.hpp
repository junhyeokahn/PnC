#pragma once

#include <Filter/filters.hpp>

class BasicAccumulation{
    public:
        BasicAccumulation();
        ~BasicAccumulation(){}

        void estimatorInitialization(
                const std::vector<double> & acc,
                const std::vector<double> & ang_vel);

        void setSensorData(
                const std::vector<double> & acc,
                const std::vector<double> & ang_vel);

        void getEstimatedState(
                Eigen::Quaternion<double> & ori,
                Eigen::Vector3d & global_ang_vel){
            ori = global_ori_quat_;
            global_ang_vel = global_ang_vel_;
        }

        void getEstimatedState(
                Eigen::Vector3d & _global_ori_ypr,
                Eigen::Vector3d & _global_ang_ypr_dot) {
            _global_ori_ypr = global_ori_ypr_;
            _global_ang_ypr_dot = global_ori_ypr_dot_;
        }

        Eigen::Vector3d _so3_to_euler_zyx_dot(const Eigen::Vector3d & _global_ori_ypr,
                                              const Eigen::Vector3d & _global_ang_vel);

    protected:
        double cutoff_freq_;
        std::vector<filter*> filtered_acc_;

        void _InitIMUOrientationEstimateFromGravity();
        Eigen::Quaternion<double> global_ori_quat_;
        Eigen::Vector3d global_ori_ypr_;
        Eigen::Vector3d global_ori_ypr_dot_;
        Eigen::Vector3d  global_ang_vel_;
};
