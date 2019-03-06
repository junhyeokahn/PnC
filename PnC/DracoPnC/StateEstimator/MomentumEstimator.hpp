#pragma once

#include <vector>

#include <PnC/DracoPnC/DracoDefinition.hpp>
#include <PnC/DracoPnC/DracoStateProvider.hpp>
#include <PnC/Filters/Basic/filters.hpp>
#include <PnC/Filters/Kalman/ExtendedKalmanFilter.hpp>
#include <PnC/Filters/Kalman/LinearizedMeasurementModel.hpp>
#include <PnC/Filters/Kalman/LinearizedSystemModel.hpp>
#include <PnC/Filters/Kalman/UnscentedKalmanFilter.hpp>
#include <PnC/RobotSystem/RobotSystem.hpp>

class MomentumDynamicsState : public Kalman::Vector<double, 9> {
   public:
    KALMAN_VECTOR(MomentumDynamicsState, double, 9)
    static constexpr size_t CX = 0;
    static constexpr size_t CY = 1;
    static constexpr size_t CZ = 2;
    static constexpr size_t LX = 3;
    static constexpr size_t LY = 4;
    static constexpr size_t LZ = 5;
    static constexpr size_t KX = 6;
    static constexpr size_t KY = 7;
    static constexpr size_t KZ = 8;

    double c_x() const { return (*this)[CX]; }
    double c_y() const { return (*this)[CY]; }
    double c_z() const { return (*this)[CZ]; }
    double l_x() const { return (*this)[LX]; }
    double l_y() const { return (*this)[LY]; }
    double l_z() const { return (*this)[LZ]; }
    double k_x() const { return (*this)[KX]; }
    double k_y() const { return (*this)[KY]; }
    double k_z() const { return (*this)[KZ]; }

    double& c_x() { return (*this)[CX]; }
    double& c_y() { return (*this)[CY]; }
    double& c_z() { return (*this)[CZ]; }
    double& l_x() { return (*this)[LX]; }
    double& l_y() { return (*this)[LY]; }
    double& l_z() { return (*this)[LZ]; }
    double& k_x() { return (*this)[KX]; }
    double& k_y() { return (*this)[KY]; }
    double& k_z() { return (*this)[KZ]; }
};

class MomentumDynamicsControl : public Kalman::Vector<double, 12> {
   public:
    KALMAN_VECTOR(MomentumDynamicsControl, double, 12)
    static constexpr size_t RTX = 0;
    static constexpr size_t RTY = 1;
    static constexpr size_t RTZ = 2;
    static constexpr size_t RFX = 3;
    static constexpr size_t RFY = 4;
    static constexpr size_t RFZ = 5;
    static constexpr size_t LTX = 6;
    static constexpr size_t LTY = 7;
    static constexpr size_t LTZ = 8;
    static constexpr size_t LFX = 9;
    static constexpr size_t LFY = 10;
    static constexpr size_t LFZ = 11;

    double r_tx() const { return (*this)[RTX]; }
    double r_ty() const { return (*this)[RTY]; }
    double r_tz() const { return (*this)[RTZ]; }
    double r_fx() const { return (*this)[RFX]; }
    double r_fy() const { return (*this)[RFY]; }
    double r_fz() const { return (*this)[RFZ]; }
    double l_tx() const { return (*this)[LTX]; }
    double l_ty() const { return (*this)[LTY]; }
    double l_tz() const { return (*this)[LTZ]; }
    double l_fx() const { return (*this)[LFX]; }
    double l_fy() const { return (*this)[LFY]; }
    double l_fz() const { return (*this)[LFZ]; }

    double& r_tx() { return (*this)[RTX]; }
    double& r_ty() { return (*this)[RTY]; }
    double& r_tz() { return (*this)[RTZ]; }
    double& r_fx() { return (*this)[RFX]; }
    double& r_fy() { return (*this)[RFY]; }
    double& r_fz() { return (*this)[RFZ]; }
    double& l_tx() { return (*this)[LTX]; }
    double& l_ty() { return (*this)[LTY]; }
    double& l_tz() { return (*this)[LTZ]; }
    double& l_fx() { return (*this)[LFX]; }
    double& l_fy() { return (*this)[LFY]; }
    double& l_fz() { return (*this)[LFZ]; }
};

class MomentumDynamicsModel
    : public Kalman::LinearizedSystemModel<MomentumDynamicsState,
                                           MomentumDynamicsControl,
                                           Kalman::StandardBase> {
   public:
    typedef MomentumDynamicsState S;
    typedef MomentumDynamicsControl C;
    MomentumDynamicsModel(RobotSystem* robot) {
        robot_ = robot;
        sp_ = DracoStateProvider::getStateProvider(robot_);
        m_ = robot_->getRobotMass();
    }
    S f(const S& x, const C& u) const {
        // dynamics
        S xdot;
        Eigen::Vector3d r_p;
        Eigen::Vector3d l_p;
        r_p = robot_->getBodyNodeIsometry(DracoBodyNode::rAnkle).translation() +
              sp_->global_pos_local;
        l_p = robot_->getBodyNodeIsometry(DracoBodyNode::lAnkle).translation() +
              sp_->global_pos_local;
        xdot.c_x() = x.l_x() / m_;
        xdot.c_y() = x.l_y() / m_;
        xdot.c_z() = x.l_z() / m_;
        xdot.l_x() = u.r_fx() + u.l_fx();
        xdot.l_y() = u.r_fy() + u.l_fy();
        xdot.l_z() = u.r_fz() + u.l_fz();
        xdot.k_x() = (r_p(1) - x.c_y()) * u.r_fz() -
                     (r_p(2) - x.c_z()) * u.r_fy() +
                     (l_p(1) - x.c_y()) * u.l_fz() -
                     (l_p(2) - x.c_z()) * u.l_fy() + u.r_tx() + u.l_tx();
        xdot.k_y() = (r_p(2) - x.c_z()) * u.r_fx() -
                     (r_p(0) - x.c_x()) * u.r_fz() +
                     (l_p(2) - x.c_z()) * u.l_fx() -
                     (l_p(0) - x.c_x()) * u.l_fz() + u.r_ty() + u.l_ty();
        xdot.k_z() = (r_p(0) - x.c_x()) * u.r_fy() -
                     (r_p(1) - x.c_y()) * u.r_fx() +
                     (l_p(0) - x.c_x()) * u.l_fy() -
                     (l_p(1) - x.c_y()) * u.l_fx() + u.r_tz() + u.l_tz();

        S xnext;
        xnext.c_x() = x.c_x() + xdot.c_x() * DracoAux::ServoRate;
        xnext.c_y() = x.c_y() + xdot.c_y() * DracoAux::ServoRate;
        xnext.c_z() = x.c_z() + xdot.c_z() * DracoAux::ServoRate;
        xnext.l_x() = x.l_x() + xdot.l_x() * DracoAux::ServoRate;
        xnext.l_y() = x.l_y() + xdot.l_y() * DracoAux::ServoRate;
        xnext.l_z() = x.l_z() + xdot.l_z() * DracoAux::ServoRate;
        xnext.k_x() = x.k_x() + xdot.k_x() * DracoAux::ServoRate;
        xnext.k_y() = x.k_y() + xdot.k_y() * DracoAux::ServoRate;
        xnext.k_z() = x.k_z() + xdot.k_z() * DracoAux::ServoRate;

        return xnext;
    }

   protected:
    double m_;
    RobotSystem* robot_;
    DracoStateProvider* sp_;
    void updateJacobians(const S& x, const C& u) {
        // jacobian F, W
        this->F(S::CX, S::CX) = 1;
        this->F(S::CY, S::CY) = 1;
        this->F(S::CZ, S::CZ) = 1;
        this->F(S::LX, S::LX) = 1;
        this->F(S::LY, S::LY) = 1;
        this->F(S::LZ, S::LZ) = 1;
        this->F(S::KX, S::KX) = 1;
        this->F(S::KY, S::KY) = 1;
        this->F(S::KZ, S::KZ) = 1;

        this->F(S::CX, S::LX) = 1 / m_;
        this->F(S::CY, S::LY) = 1 / m_;
        this->F(S::CZ, S::LZ) = 1 / m_;

        this->F(S::KX, S::CY) = -u.r_fz() - u.l_fz();
        this->F(S::KX, S::CZ) = +u.r_fy() + u.l_fy();
        this->F(S::KY, S::CX) = +u.r_fz() + u.l_fz();
        this->F(S::KY, S::CZ) = -u.r_fx() - u.l_fx();
        this->F(S::KZ, S::CX) = -u.r_fy() - u.l_fy();
        this->F(S::KZ, S::CY) = +u.r_fx() + u.l_fx();

        Eigen::Vector3d r_p;
        Eigen::Vector3d l_p;
        r_p = robot_->getBodyNodeIsometry(DracoBodyNode::rAnkle).translation() +
              sp_->global_pos_local;
        l_p = robot_->getBodyNodeIsometry(DracoBodyNode::lAnkle).translation() +
              sp_->global_pos_local;

        this->W.setZero();

        this->W(S::LX, C::RFX) = 1;
        this->W(S::LY, C::RFY) = 1;
        this->W(S::LZ, C::RFZ) = 1;
        this->W(S::LX, C::LFX) = 1;
        this->W(S::LY, C::LFY) = 1;
        this->W(S::LZ, C::LFZ) = 1;

        this->W(S::KX, C::RFY) = -(r_p(2) - x.c_z());
        this->W(S::KX, C::RFZ) = +(r_p(1) - x.c_y());
        this->W(S::KY, C::RFX) = +(r_p(2) - x.c_z());
        this->W(S::KY, C::RFZ) = -(r_p(0) - x.c_x());
        this->W(S::KZ, C::RFX) = -(r_p(1) - x.c_y());
        this->W(S::KZ, C::RFY) = +(r_p(0) - x.c_x());

        this->W(S::KX, C::LFY) = -(l_p(2) - x.c_z());
        this->W(S::KX, C::LFZ) = +(l_p(1) - x.c_y());
        this->W(S::KY, C::LFX) = +(l_p(2) - x.c_z());
        this->W(S::KY, C::LFZ) = -(l_p(0) - x.c_x());
        this->W(S::KZ, C::LFX) = -(l_p(1) - x.c_y());
        this->W(S::KZ, C::LFY) = +(l_p(0) - x.c_x());

        this->W(S::KX, C::RTX) = 1;
        this->W(S::KY, C::RTY) = 1;
        this->W(S::KZ, C::RTZ) = 1;
        this->W(S::KX, C::LTX) = 1;
        this->W(S::KY, C::LTY) = 1;
        this->W(S::KZ, C::LTZ) = 1;
    }
};

class MomentumMeasurement : public Kalman::Vector<double, 6> {
   public:
    KALMAN_VECTOR(MomentumMeasurement, double, 6)
    static constexpr size_t CX = 0;
    static constexpr size_t CY = 1;
    static constexpr size_t CZ = 2;
    static constexpr size_t KX = 3;
    static constexpr size_t KY = 4;
    static constexpr size_t KZ = 5;

    double c_x() const { return (*this)[CX]; }
    double c_y() const { return (*this)[CY]; }
    double c_z() const { return (*this)[CZ]; }
    double k_x() const { return (*this)[KX]; }
    double k_y() const { return (*this)[KY]; }
    double k_z() const { return (*this)[KZ]; }

    double& c_x() { return (*this)[CX]; }
    double& c_y() { return (*this)[CY]; }
    double& c_z() { return (*this)[CZ]; }
    double& k_x() { return (*this)[KX]; }
    double& k_y() { return (*this)[KY]; }
    double& k_z() { return (*this)[KZ]; }
};

class MomentumObservationModel
    : public Kalman::LinearizedMeasurementModel<
          MomentumDynamicsState, MomentumMeasurement, Kalman::StandardBase> {
   public:
    typedef MomentumDynamicsState S;
    typedef MomentumMeasurement M;

    MomentumObservationModel(RobotSystem* robot) {
        robot_ = robot;
        sp_ = DracoStateProvider::getStateProvider(robot_);
    }

    M h(const S& x) const {
        // observation
        Eigen::Vector3d com = robot_->getCoMPosition() + sp_->global_pos_local;
        Eigen::VectorXd cm = robot_->getCentroidMomentum();
        M measurement;
        measurement.c_x() = com(0);
        measurement.c_y() = com(1);
        measurement.c_z() = com(2);
        measurement.k_x() = cm(0);
        measurement.k_y() = cm(1);
        measurement.k_z() = cm(2);

        return measurement;
    }

   protected:
    RobotSystem* robot_;
    DracoStateProvider* sp_;
    void updateJacobians(const S& x) {
        // jacobian H, V
        this->H.setZero();
        this->H(M::CX, S::CX) = 1;
        this->H(M::CY, S::CY) = 1;
        this->H(M::CZ, S::CZ) = 1;
        this->H(M::KX, S::KX) = 1;
        this->H(M::KY, S::KY) = 1;
        this->H(M::KZ, S::KZ) = 1;

        this->V.setZero();
    }
};

class MomentumEstimator {
   public:
    MomentumEstimator(RobotSystem*);
    virtual ~MomentumEstimator();

    void Initialization(const Eigen::VectorXd rankle_ft,
                        const Eigen::VectorXd lankle_ft);
    void Update(const Eigen::VectorXd rankle_ft,
                const Eigen::VectorXd lankle_ft);
    // return [c, l, k] \in R^{9}
    Eigen::VectorXd GetEstimatedState() { return estimated_state_; };

   private:
    std::vector<filter*> rankle_ft_filter_;
    std::vector<filter*> lankle_ft_filter_;

    RobotSystem* robot_;

    MomentumDynamicsModel* dyn_model_;
    MomentumObservationModel* obs_model_;
    Kalman::ExtendedKalmanFilter<MomentumDynamicsState>* ekf_;

    Eigen::VectorXd estimated_state_;

    DracoStateProvider* sp_;
};
