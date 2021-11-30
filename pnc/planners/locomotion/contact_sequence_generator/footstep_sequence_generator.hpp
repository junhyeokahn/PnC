#pragma once

#include <iostream>

#include <Eigen/Dense>
#include <vector>

#include "pnc/robot_system/centroid_model.hpp"

class FootstepSequenceGenerator {
   public:
    FootstepSequenceGenerator(){};
    virtual ~FootstepSequenceGenerator(){};

    // Parameters Setting
    void SetLocalRightFootPos(const Eigen::Vector3d &pos){
        r_ft_displacement_ = pos; // note that the r_ft_displacement_[1] should be negative
    }
    void SetLocalLeftFootPos(const Eigen::Vector3d &pos){
        l_ft_displacement_ = pos;
    }
    void SetLocalRightFootOrientation(const Eigen::Vector4d &ori) {
        r_ft_ori_.w() = ori[0];
        r_ft_ori_.x() = ori[1];
        r_ft_ori_.y() = ori[2];
        r_ft_ori_.z() = ori[3];
    }
    void SetLocalLeftFootOrientation(const Eigen::Vector4d &ori) {
        l_ft_ori_.w() = ori[0];
        l_ft_ori_.x() = ori[1];
        l_ft_ori_.y() = ori[2];
        l_ft_ori_.z() = ori[3];
    }
    // header description

    // Update Function
    void Update(const CentroidModel::EEfID &curr_stance,
                const Eigen::Isometry3d &curr_rf_iso,
                const Eigen::Isometry3d &curr_lf_iso) {
        curr_rf_iso_ = curr_rf_iso;
        curr_lf_iso_ = curr_lf_iso;
        curr_stance_ = curr_stance;
    }

    // Get Function
    void GetNextFootStep(CentroidModel::EEfID &next_stance,
                         Eigen::Isometry3d &next_foot_iso);
    void GetFinalFootStep(CentroidModel::EEfID &next_stance,
                          Eigen::Isometry3d &next_foot_iso);

   private:
    Eigen::Vector3d r_ft_displacement_;
    Eigen::Vector3d l_ft_displacement_;
    Eigen::Quaternion<double> r_ft_ori_;
    Eigen::Quaternion<double> l_ft_ori_;

    // Current State
    Eigen::Isometry3d curr_rf_iso_;
    Eigen::Isometry3d curr_lf_iso_;
    CentroidModel::EEfID curr_stance_;
};
