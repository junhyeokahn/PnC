#include <pnc/planners/locomotion/contact_sequence_generator/footstep_sequence_generator.hpp>

void FootstepSequenceGenerator::setLocalFootDisplacement(double _foot_length
                            double _foot_width
                            double _foot_height
                            Eigen::Quaternion<double> _foot_ori){
    foot_legnth_ = _foot_legnth;
    foot_width_ = _foot_width;
    foot_height_ = _foot_height;
    foot_ori_ = _foot_ori;
    };

void FootstepSequenceGenerator::getNextFootStep(CentroidModel::EEfID& _next_stance_foot, 
                                                Eigen::Isometry3d& _next_foot_iso){
    if (curr_stance_foot_ == CentroidModel::EEfID::lfoot) {
       _next_stance_foot = CentroidModel::EEfID::rfoot;

       Eigen::Isometry3d local_foot_inc = Eigen::Isometry3d::Identity();
       Eigen::Isometry3d global_foot_inc = Eigen::Isometry3d::Identity();

       local_foot_inc.translation() << foot_length_, -foot_width_, foot_height_;
       local_foot.inc.linear() = foot_ori_.toRotationMatrix();

       global_foot_inc.translation() = curr_lf_iso_.translation() + curr_lf_iso_.linear() * local_foot_inc.translation();





    }}
