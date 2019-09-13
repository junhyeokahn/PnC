#include <PnC/RobotSystem/RobotSystem.hpp>
#include <PnC/DracoPnC/DracoDefinition.hpp>
#include <PnC/DracoPnC/DracoInterface.hpp>
#include <PnC/DracoPnC/DracoStateEstimator.hpp>
#include <PnC/DracoPnC/DracoStateProvider.hpp>
#include <Utils/IO/IOUtilities.hpp>

DracoStateEstimator::DracoStateEstimator(RobotSystem* robot) {
    myUtils::pretty_constructor(1, "Draco State Estimator");

    robot_ = robot;
    sp_ = DracoStateProvider::getStateProvider(robot_);
    curr_config_ = Eigen::VectorXd::Zero(Draco::n_dof);
    curr_qdot_ = Eigen::VectorXd::Zero(Draco::n_dof);
}

DracoStateEstimator::~DracoStateEstimator() {}

void DracoStateEstimator::Initialization(DracoSensorData* data) {
    _JointUpdate(data);
    _ConfigurationAndModelUpdate();
    sp_->jpos_ini = curr_config_.segment(Draco::n_vdof, Draco::n_adof);
    _FootContactUpdate(data);
    sp_->saveCurrentData();
}

void DracoStateEstimator::Update(DracoSensorData* data) {
    _JointUpdate(data);
    _ConfigurationAndModelUpdate();
    _FootContactUpdate(data);
    sp_->saveCurrentData();
}

void DracoStateEstimator::_JointUpdate(DracoSensorData* data) {
    curr_config_.setZero();
    curr_qdot_.setZero();
    for (int i = 0; i < Draco::n_vdof; ++i) {
        curr_config_[i] = data->virtual_q[i];
        curr_qdot_[i] = data->virtual_qdot[i];
    }
    for (int i(0); i < Draco::n_adof; ++i) {
        curr_config_[Draco::n_vdof + i] = data->q[i];
        curr_qdot_[Draco::n_vdof + i] = data->qdot[i];
    }

    sp_->l_rf = data->lf_wrench;
    sp_->r_rf = data->rf_wrench;
}

void DracoStateEstimator::_ConfigurationAndModelUpdate() {
    robot_->updateSystem(curr_config_, curr_qdot_, true);

    sp_->q = curr_config_;
    sp_->qdot = curr_qdot_;
}

void DracoStateEstimator::_FootContactUpdate(DracoSensorData* data) {
    if (data->rfoot_contact)
        sp_->b_rfoot_contact = 1;
    else
        sp_->b_rfoot_contact = 0;
    if (data->lfoot_contact)
        sp_->b_lfoot_contact = 1;
    else
        sp_->b_lfoot_contact = 0;
}
