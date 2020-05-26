#include <PnC/RobotSystem/RobotSystem.hpp>
#include <PnC/ValkyriePnC/ValkyrieDefinition.hpp>
#include <PnC/ValkyriePnC/ValkyrieInterface.hpp>
#include <PnC/ValkyriePnC/ValkyrieStateEstimator.hpp>
#include <PnC/ValkyriePnC/ValkyrieStateProvider.hpp>
#include <Utils/IO/IOUtilities.hpp>

ValkyrieStateEstimator::ValkyrieStateEstimator(RobotSystem* robot) {
    myUtils::pretty_constructor(1, "Valkyrie State Estimator");

    robot_ = robot;
    sp_ = ValkyrieStateProvider::getStateProvider(robot_);
    curr_config_ = Eigen::VectorXd::Zero(Valkyrie::n_dof);
    curr_qdot_ = Eigen::VectorXd::Zero(Valkyrie::n_dof);
}

ValkyrieStateEstimator::~ValkyrieStateEstimator() {}

void ValkyrieStateEstimator::Initialization(ValkyrieSensorData* data) {
    _JointUpdate(data);
    _ConfigurationAndModelUpdate();
    sp_->jpos_ini = curr_config_.segment(Valkyrie::n_vdof, Valkyrie::n_adof);
    _FootContactUpdate(data);
    _UpdateDCM();
    sp_->saveCurrentData();
}

void ValkyrieStateEstimator::Update(ValkyrieSensorData* data) {
    _JointUpdate(data);
    _ConfigurationAndModelUpdate();
    _FootContactUpdate(data);
    _UpdateDCM();
    sp_->saveCurrentData();
}

void ValkyrieStateEstimator::_JointUpdate(ValkyrieSensorData* data) {
    curr_config_.setZero();
    curr_qdot_.setZero();
    for (int i = 0; i < Valkyrie::n_vdof; ++i) {
        curr_config_[i] = data->virtual_q[i];
        curr_qdot_[i] = data->virtual_qdot[i];
    }
    for (int i(0); i < Valkyrie::n_adof; ++i) {
        curr_config_[Valkyrie::n_vdof + i] = data->q[i];
        curr_qdot_[Valkyrie::n_vdof + i] = data->qdot[i];
    }

    sp_->l_rf = data->lf_wrench;
    sp_->r_rf = data->rf_wrench;
}

void ValkyrieStateEstimator::_ConfigurationAndModelUpdate() {
    robot_->updateSystem(curr_config_, curr_qdot_, true);

    sp_->q = curr_config_;
    sp_->qdot = curr_qdot_;
}

void ValkyrieStateEstimator::_FootContactUpdate(ValkyrieSensorData* data) {
    if (data->rfoot_contact)
        sp_->b_rfoot_contact = 1;
    else
        sp_->b_rfoot_contact = 0;
    if (data->lfoot_contact)
        sp_->b_lfoot_contact = 1;
    else
        sp_->b_lfoot_contact = 0;
}

void ValkyrieStateEstimator::_UpdateDCM(){
    sp_->com_pos = robot_->getCoMPosition();
    sp_->com_vel = robot_->getCoMVelocity();
    sp_->dcm_omega = sqrt(9.81/sp_->com_pos[2]);

    sp_->prev_dcm = sp_->dcm;
    sp_->dcm = robot_->getCoMPosition() + sp_->com_vel /sp_->dcm_omega; 

    double alpha_vel = 0.1;
    sp_->dcm_vel = alpha_vel*((sp_->dcm - sp_->prev_dcm)/ValkyrieAux::servo_rate) +
                   (1.0 - alpha_vel)*sp_->dcm_vel;
}