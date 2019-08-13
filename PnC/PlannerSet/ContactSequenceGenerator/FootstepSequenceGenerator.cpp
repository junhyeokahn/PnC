#include <PnC/PlannerSet/ContactSequenceGenerator/FootstepSequenceGenerator.hpp>

void FootstepSequenceGenerator::GetNextFootStep(
    CentroidModel::EEfID &next_stance, Eigen::VectorXd &next_foot_pos) {
    next_foot_pos = Eigen::VectorXd::Zero(3);

    if (curr_stance_ == CentroidModel::EEfID::rightFoot) {
        next_stance = CentroidModel::EEfID::leftFoot;
        next_foot_pos[0] = curr_rf_pos_[0] + ft_length_;
        next_foot_pos[1] = curr_rf_pos_[1] + ft_spread_;
        // update for the next step
        curr_stance_ = CentroidModel::EEfID::leftFoot;
        curr_lf_pos_ = next_foot_pos;
    } else {
        next_stance = CentroidModel::EEfID::rightFoot;
        next_foot_pos[0] = curr_lf_pos_[0] + ft_length_;
        next_foot_pos[1] = curr_lf_pos_[1] - ft_spread_;
        // update for the next step
        curr_stance_ = CentroidModel::EEfID::rightFoot;
        curr_rf_pos_ = next_foot_pos;
    }
}
