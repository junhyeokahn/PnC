#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <vector>

#include "pnc/robot_system/centroid_model.hpp"

class FootstepSequenceGenerator{

    public:
    FootstepSequenceGenerator(){};
    virtual ~FootstepSequenceGenerator(){};

    void setLocalFootDisplacement(double _foot_length
                            double _foot_width
                            double _foot_height
                            Eigen::Quaternion<double> _foot_ori){};


    private:
    double foot_length_;
    double foot_width_;
    double foot_height_;
    Eigen::Quaternion<double> foot_ori_;

    Eigen::Isometry3d curr_lf_iso_;
    Eigen::Isometry3d curr_rf_iso_;
    CentroidModel::EEfID curr_stance_foot_;
};
