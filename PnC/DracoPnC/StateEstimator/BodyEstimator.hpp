#pragma once

#include <Filters/Basic/filters.hpp>

class DracoMoCapManager;
class RobotSystem;
class DracoStateProvider;

class BodyEstimator{
    public:
        friend class DracoMoCapManager;

        BodyEstimator(RobotSystem*);
        ~BodyEstimator();

        void Initialization(const Eigen::Quaternion<double> & body_ori);
        void Update();

        void getMoCapBodyOri(Eigen::Quaternion<double> & quat);
        void getMoCapBodyVel(Eigen::Vector3d & body_vel);
        void getMoCapBodyPos(const Eigen::Quaternion<double>& body_ori, 
                Eigen::Vector3d & local_body_pos);

    protected:
        DracoMoCapManager* mocap_manager_;
        DracoStateProvider* sp_;

        std::vector<filter*> vel_filter_;
        Eigen::Vector3d body_led_vel_;

        RobotSystem* robot_;
};
