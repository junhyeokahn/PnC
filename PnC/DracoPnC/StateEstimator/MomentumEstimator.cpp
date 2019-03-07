#include <PnC/DracoPnC/StateEstimator/MomentumEstimator.hpp>
#include <Utils/IO/DataManager.hpp>

MomentumEstimator::MomentumEstimator(RobotSystem* robot) {
    myUtils::pretty_constructor(2, "Momentum Estimator");

    robot_ = robot;
    sp_ = DracoStateProvider::getStateProvider(robot_);
    Eigen::VectorXd ft_limit = Eigen::VectorXd::Zero(6);
    ft_limit << 50., 50., 50., 300., 300., 600.;
    for (int i = 0; i < 6; ++i) {
        /*        rankle_ft_filter_.push_back(*/
        // new digital_lp_filter(2. * M_PI * 50, DracoAux::ServoRate));
        // lankle_ft_filter_.push_back(
        /*new digital_lp_filter(2. * M_PI * 50, DracoAux::ServoRate));*/

        rankle_ft_filter_.push_back(
            new AverageFilter(DracoAux::ServoRate, 0.01, ft_limit[i]));
        lankle_ft_filter_.push_back(
            new AverageFilter(DracoAux::ServoRate, 0.01, ft_limit[i]));
    }

    dyn_model_ = new MomentumDynamicsModel(robot_);
    dyn_model_->setCovariance(1 * dyn_model_->getCovariance());
    obs_model_ = new MomentumObservationModel(robot_);
    obs_model_->setCovariance(1 * obs_model_->getCovariance());
    ekf_ = new Kalman::ExtendedKalmanFilter<MomentumDynamicsState>();

    estimated_state_ = Eigen::VectorXd::Zero(9);

    // For Debug Purpose >>>>>>>>>>>>>
    DataManager* data_manager = DataManager::GetDataManager();
    debug_upd_ = Eigen::VectorXd::Zero(9);
    debug_true_ = Eigen::VectorXd::Zero(9);
    debug_obs_ = Eigen::VectorXd::Zero(9);
    debug_pred_ = Eigen::VectorXd::Zero(9);
    debug_rfoot_ati_raw_ = Eigen::VectorXd::Zero(6);
    debug_lfoot_ati_raw_ = Eigen::VectorXd::Zero(6);
    debug_rfoot_ati_ = Eigen::VectorXd::Zero(6);
    debug_lfoot_ati_ = Eigen::VectorXd::Zero(6);
    data_manager->RegisterData(&debug_pred_, VECT, "debug_pred", 9);
    data_manager->RegisterData(&debug_obs_, VECT, "debug_obs", 9);
    data_manager->RegisterData(&debug_upd_, VECT, "debug_upd", 9);
    data_manager->RegisterData(&debug_true_, VECT, "debug_true", 9);
    data_manager->RegisterData(&debug_rfoot_ati_raw_, VECT,
                               "debug_rfoot_ati_raw", 6);
    data_manager->RegisterData(&debug_lfoot_ati_raw_, VECT,
                               "debug_lfoot_ati_raw", 6);
    data_manager->RegisterData(&debug_rfoot_ati_, VECT, "debug_rfoot_ati", 6);
    data_manager->RegisterData(&debug_lfoot_ati_, VECT, "debug_lfoot_ati", 6);
    // <<<<<<<<<<<<< For Debug Purpose
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
    // TEST
    // std::cout << "dyn cov" << std::endl;
    // static Eigen::MatrixXd init_cov = dyn_model_->getCovariance();
    // dym_model_->setCovariance()
    // std::cout << dyn_model_->getCovariance() << std::endl;
    // exit(0);
    // TEST
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

    // For Debug Purpose >>>>>>>>>>>>>
    debug_pred_[0] = s1.c_x();
    debug_pred_[1] = s1.c_y();
    debug_pred_[2] = s1.c_z();
    debug_pred_[3] = s1.l_x();
    debug_pred_[4] = s1.l_y();
    debug_pred_[5] = s1.l_z();
    debug_pred_[6] = s1.k_x();
    debug_pred_[7] = s1.k_y();
    debug_pred_[8] = s1.k_z();
    debug_rfoot_ati_ = rankle_ft_output;
    debug_lfoot_ati_ = lankle_ft_output;
    debug_rfoot_ati_raw_ = rankle_ft;
    debug_lfoot_ati_raw_ = lankle_ft;
    // <<<<<<<<<<<<< For Debug Purpose

    // ekf update
    MomentumDynamicsState s2;
    MomentumMeasurement o;

    Eigen::Vector3d com = robot_->getCoMPosition() + sp_->global_pos_local;
    Eigen::VectorXd cm = robot_->getCentroidMomentum();

    // For Debug Purpose >>>>>>>>>>>>>>>>>
    std::normal_distribution<double> noise(0, 1);
    double c_noise = 0.02;
    double l_noise = 0.6;
    std::default_random_engine generator;
    generator.seed(std::chrono::system_clock::now().time_since_epoch().count());

    o.c_x() = com(0) + c_noise * noise(generator);
    o.c_y() = com(1) + c_noise * noise(generator);
    o.c_z() = com(2) + c_noise * noise(generator);
    o.k_x() = cm(0) + l_noise * noise(generator);
    o.k_y() = cm(1) + l_noise * noise(generator);
    o.k_z() = cm(2) + l_noise * noise(generator);

    // o.c_x() = com(0);
    // o.c_y() = com(1);
    // o.c_z() = com(2);
    // o.k_x() = cm(0);
    // o.k_y() = cm(1);
    // o.k_z() = cm(2);

    // <<<<<<<<<<<<< For Debug Purpose
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

    // For Debug Purpose >>>>>>>>>>>>>>>>>
    debug_upd_ = estimated_state_;
    debug_obs_[0] = o.c_x();
    debug_obs_[1] = o.c_y();
    debug_obs_[2] = o.c_z();
    debug_obs_[6] = o.k_x();
    debug_obs_[7] = o.k_y();
    debug_obs_[8] = o.k_z();

    debug_true_[0] = com(0);
    debug_true_[1] = com(1);
    debug_true_[2] = com(2);
    debug_true_[3] = cm(3);
    debug_true_[4] = cm(4);
    debug_true_[5] = cm(5);
    debug_true_[6] = cm(0);
    debug_true_[7] = cm(1);
    debug_true_[8] = cm(2);

    // <<<<<<<<<<<<< For Debug Purpose
}
