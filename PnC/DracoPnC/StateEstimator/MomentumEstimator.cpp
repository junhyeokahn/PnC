#include <PnC/DracoPnC/StateEstimator/MomentumEstimator.hpp>

MomentumEstimator::MomentumEstimator(RobotSystem* robot) {
    myUtils::pretty_constructor(2, "Momentum Estimator");

    robot_ = robot;
    sp_ = DracoStateProvider::getStateProvider(robot_);
    for (int i = 0; i < 6; ++i) {
        rankle_ft_filter_.push_back(
            new digital_lp_filter(2. * M_PI * 50, DracoAux::ServoRate));
        lankle_ft_filter_.push_back(
            new digital_lp_filter(2. * M_PI * 50, DracoAux::ServoRate));
    }

    dyn_model_ = new MomentumDynamicsModel(robot_);
    obs_model_ = new MomentumObservationModel(robot_);
    ekf_ = new Kalman::ExtendedKalmanFilter<MomentumDynamicsState>();

    estimated_state_ = Eigen::VectorXd::Zero(9);
}

MomentumEstimator::~MomentumEstimator() {
    for (int i = 0; i < 6; ++i) {
        delete rankle_ft_filter_[i];
        delete lankle_ft_filter_[i];
    }
    delete dyn_model_;
    delete obs_model_;
    delete ekf_;
}

void MomentumEstimator::Initialization(const Eigen::VectorXd rankle_ft,
                                       const Eigen::VectorXd lankle_ft) {
    for (int i = 0; i < 6; ++i) {
        rankle_ft_filter_[i]->input(rankle_ft[i]);
        lankle_ft_filter_[i]->input(lankle_ft[i]);
    }

    Eigen::Vector3d com = robot_->getCoMPosition() + sp_->global_pos_local;
    Eigen::VectorXd cm = robot_->getCentroidMomentum();
    MomentumDynamicsState s;
    s.c_x() = com(0);
    s.c_y() = com(1);
    s.c_z() = com(2);
    s.l_x() = cm(3);
    s.l_y() = cm(4);
    s.l_z() = cm(5);
    s.k_x() = cm(0);
    s.k_y() = cm(1);
    s.k_z() = cm(2);
    ekf_->init(s);
}

void MomentumEstimator::Update(const Eigen::VectorXd rankle_ft,
                               const Eigen::VectorXd lankle_ft) {
    Eigen::VectorXd rankle_ft_output = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd lankle_ft_output = Eigen::VectorXd::Zero(6);
    for (int i = 0; i < 6; ++i) {
        rankle_ft_filter_[i]->input(rankle_ft[i]);
        lankle_ft_filter_[i]->input(lankle_ft[i]);
        rankle_ft_output[i] = rankle_ft_filter_[i]->output();
        lankle_ft_output[i] = lankle_ft_filter_[i]->output();
    }

    // ekf predict
    MomentumDynamicsState s1;
    MomentumDynamicsControl u;
    u.r_tx() = rankle_ft_output(0);
    u.r_ty() = rankle_ft_output(1);
    u.r_tz() = rankle_ft_output(2);
    u.r_fx() = rankle_ft_output(3);
    u.r_fy() = rankle_ft_output(4);
    u.r_fz() = rankle_ft_output(5);

    u.l_tx() = lankle_ft_output(0);
    u.l_ty() = lankle_ft_output(1);
    u.l_tz() = lankle_ft_output(2);
    u.l_fx() = lankle_ft_output(3);
    u.l_fy() = lankle_ft_output(4);
    u.l_fz() = lankle_ft_output(5);

    s1 = ekf_->predict(*dyn_model_, u);

    // ekf update
    MomentumDynamicsState s2;
    MomentumMeasurement o;

    Eigen::Vector3d com = robot_->getCoMPosition() + sp_->global_pos_local;
    Eigen::VectorXd cm = robot_->getCentroidMomentum();
    o.c_x() = com(0);
    o.c_y() = com(1);
    o.c_z() = com(2);
    o.k_x() = cm(0);
    o.k_y() = cm(1);
    o.k_z() = cm(2);
    s2 = ekf_->update(*obs_model_, o);

    estimated_state_[0] = s2.c_x();
    estimated_state_[1] = s2.c_y();
    estimated_state_[2] = s2.c_z();
    estimated_state_[3] = s2.l_x();
    estimated_state_[4] = s2.l_y();
    estimated_state_[5] = s2.l_z();
    estimated_state_[6] = s2.k_x();
    estimated_state_[7] = s2.k_y();
    estimated_state_[8] = s2.k_z();
}
