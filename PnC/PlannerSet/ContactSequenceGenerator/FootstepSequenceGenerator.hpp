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

    // Update Function
    void Update(const CentroidModel::EEfID &curr_stance,
                const Eigen::VectorXd &curr_rf_pos,
                const Eigen::VectorXd &curr_lf_pos) {
        curr_rf_pos_ = curr_rf_pos;
        curr_lf_pos_ = curr_lf_pos;
        curr_stance_ = curr_stance;
    }

    // Get Function
    void GetNextFootStep(CentroidModel::EEfID &next_stance,
                         Eigen::VectorXd &next_foot_pos);

   private:
    double ft_spread_;
    double ft_length_;

    // Current State
    Eigen::VectorXd curr_rf_pos_;
    Eigen::VectorXd curr_lf_pos_;
    CentroidModel::EEfID curr_stance_;
};
