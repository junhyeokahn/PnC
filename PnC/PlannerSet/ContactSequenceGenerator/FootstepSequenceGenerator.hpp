#pragma once

#include <iostream>

#include <Eigen/Dense>
#include <vector>

#include "PnC/RobotSystem/CentroidModel.hpp"
#include "Utils/IO/IOUtilities.hpp"

class FootstepSequenceGenerator {
   public:
    FootstepSequenceGenerator(){};
    virtual ~FootstepSequenceGenerator(){};

    // Parameters Setting
    void SetFootStepWidth(double v) { ft_spread_ = v; }
    void SetFootStepLength(double v) { ft_length_ = v; }
    void SetFootStepOrientation(const Eigen::VectorXd &v) {
        ft_ori_.w() = v[0];
        ft_ori_.x() = v[1];
        ft_ori_.y() = v[2];
        ft_ori_.z() = v[3];
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
    double ft_spread_;
    double ft_length_;
    Eigen::Quaternion<double> ft_ori_;

    // Current State
    Eigen::Isometry3d curr_rf_iso_;
    Eigen::Isometry3d curr_lf_iso_;
    CentroidModel::EEfID curr_stance_;
};
