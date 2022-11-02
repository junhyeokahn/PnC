#pragma once

#include "third_party/kalman_filters/LinearizedSystemModel.hpp"

class State : public Kalman::Vector<double, 12>
{
public:
    KALMAN_VECTOR(State, double, 12);

    double base_pos_x() const {return (*this) [base_position_x]; }
    double base_pos_y() const {return (*this) [base_position_y]; }
    double base_pos_z() const {return (*this) [base_position_z]; }
    double base_vel_x() const {return (*this) [base_velocity_x]; }
    double base_vel_y() const {return (*this) [base_velocity_y]; }
    double base_vel_z() const {return (*this) [base_velocity_z]; }
    double lfoot_pos_x() const {return (*this) [lfoot_position_x]; }
    double lfoot_pos_y() const {return (*this) [lfoot_position_y]; }
    double lfoot_pos_z() const {return (*this) [lfoot_position_z]; }
    double rfoot_pos_x() const {return (*this) [rfoot_position_x]; }
    double rfoot_pos_y() const {return (*this) [rfoot_position_y]; }
    double rfoot_pos_z() const {return (*this) [rfoot_position_z]; }

    double& base_pos_x() {return (*this) [base_position_x]; }
    double& base_pos_y() {return (*this) [base_position_y]; }
    double& base_pos_z() {return (*this) [base_position_z]; }
    double& base_vel_x() {return (*this) [base_velocity_x]; }
    double& base_vel_y() {return (*this) [base_velocity_y]; }
    double& base_vel_z() {return (*this) [base_velocity_z]; }
    double& lfoot_pos_x() {return (*this) [lfoot_position_x]; }
    double& lfoot_pos_y() {return (*this) [lfoot_position_y]; }
    double& lfoot_pos_z() {return (*this) [lfoot_position_z]; }
    double& rfoot_pos_x() {return (*this) [rfoot_position_x]; }
    double& rfoot_pos_y() {return (*this) [rfoot_position_y]; }
    double& rfoot_pos_z() {return (*this) [rfoot_position_z]; }

    void initialize(const Eigen::Vector3d &base_transform,
                    const Eigen::Isometry3d &lfoot_transform,
                    const Eigen::Isometry3d &rfoot_transform) {
    base_pos_x() = base_transform.x();
    base_pos_y() = base_transform.y();
    base_pos_z() = base_transform.z();

    base_vel_x() = 0.0;
    base_vel_y() = 0.0;
    base_vel_z() = 0.0;

    lfoot_pos_x() = base_pos_x() + lfoot_transform.translation().x();
    lfoot_pos_y() = base_pos_y() + lfoot_transform.translation().y();
    lfoot_pos_z() = base_pos_z() + lfoot_transform.translation().z();

    rfoot_pos_x() = base_pos_x() + rfoot_transform.translation().x();
    rfoot_pos_y() = base_pos_y() + rfoot_transform.translation().y();
    rfoot_pos_z() = base_pos_z() + rfoot_transform.translation().z();
    }

    static constexpr size_t base_position_x = 0;
    static constexpr size_t base_position_y = 1;
    static constexpr size_t base_position_z = 2;
    static constexpr size_t base_velocity_x = 3;
    static constexpr size_t base_velocity_y = 4;
    static constexpr size_t base_velocity_z = 5;
    static constexpr size_t lfoot_position_x = 6;
    static constexpr size_t lfoot_position_y = 7;
    static constexpr size_t lfoot_position_z = 8;
    static constexpr size_t rfoot_position_x = 9;
    static constexpr size_t rfoot_position_y = 10;
    static constexpr size_t rfoot_position_z = 11;
};

class Control : public Kalman::Vector<double, 3>
{
public:
    KALMAN_VECTOR(Control, double, 3);

    double accel_measurement_x;
    double accel_measurement_y;
    double accel_measurement_z;

    double accel_meas_x() const { return accel_measurement_x; }
    double accel_meas_y() const { return accel_measurement_y; }
    double accel_meas_z() const { return accel_measurement_z; }

};

static double COV_LEVEL_LOW = 0.001;
static double COV_LEVEL_HIGH = 10.0;

//template<typename Vector3, template<class> class CovarianceBase = Kalman::StandardBase>
class FloatingBaseSystemModel : public Kalman::LinearizedSystemModel<State, Control, Kalman::StandardBase>
{
public:

    FloatingBaseSystemModel()
    {
      F.setZero();
      B.setZero();
      position_offset.setZero();
      lfoot_offset.setZero();
      rfoot_offset.setZero();
      W.setZero();
    }

    void initialize(const double &delta_t,
                    const Eigen::Vector3d &sigma_base_pos,
                    const Eigen::Vector3d &sigma_base_vel,
                    const Eigen::Vector3d &sigma_pos_lfoot,
                    const Eigen::Vector3d &sigma_pos_rfoot)
    {
      // assign non-zero elements to A and B matrices
      F.block(0, 0, 3, 3) = I;
      F.block(0, 3, 3, 3) = delta_t * I;
      F.block(3, 3, 3, 3) = I;
      F.block(6, 6, 3, 3) = I;
      F.block(9, 9, 3, 3) = I;
      B.block(0, 0, 3, 3) = 0.5 * delta_t * delta_t * I;
      B.block(3, 0, 3, 3) = delta_t * I;

      W.block(0, 0, 3, 3) = sigma_base_pos.asDiagonal();
      W.block(3, 3, 3, 3) = sigma_base_vel.asDiagonal();
      W.block(6, 6, 3, 3) = sigma_pos_lfoot.asDiagonal();
      W.block(9, 9, 3, 3) = sigma_pos_rfoot.asDiagonal();
    }

    void update_base_offset(const Eigen::Vector3d& offset)
    {
      this->position_offset = offset;
    }

    void update_lfoot_offset(const Eigen::Vector3d& offset)
    {
      this->lfoot_offset = offset;
    }

    void update_rfoot_offset(const Eigen::Vector3d& offset)
    {
      this->rfoot_offset = offset;
    }

    void reset_offsets()
    {
      this->lfoot_offset.setZero();
      this->rfoot_offset.setZero();
    }


    State f(const State& x, const Control& u) const
    {
      State x_next;

      // We perform the state prediction taking advantage of sparsity in A and B
      x_next.base_pos_x() = F(0,0) * x.base_pos_x() + F(0,3) * x.base_vel_x() + B(0,0) * u.accel_meas_x();
      x_next.base_pos_y() = F(1,1) * x.base_pos_y() + F(1,4) * x.base_vel_y() + B(1,1) * u.accel_meas_y();
      x_next.base_pos_z() = F(2,2) * x.base_pos_z() + F(2,5) * x.base_vel_z() + B(2,2) * u.accel_meas_z();
      x_next.base_vel_x() = F(3, 3) * x.base_vel_x() + B(3,0) * u.accel_meas_x();
      x_next.base_vel_y() = F(4, 4) * x.base_vel_y() + B(4,1) * u.accel_meas_y();
      x_next.base_vel_z() = F(5, 5) * x.base_vel_z() + B(5,2) * u.accel_meas_z();
      x_next.lfoot_pos_x() = F(6, 6) * x.lfoot_pos_x() + lfoot_offset.x();
      x_next.lfoot_pos_y() = F(7, 7) * x.lfoot_pos_y() + lfoot_offset.y();
      x_next.lfoot_pos_z() = F(8, 8) * x.lfoot_pos_z() + lfoot_offset.z();
      x_next.rfoot_pos_x() = F(9, 9) * x.rfoot_pos_x() + rfoot_offset.x();
      x_next.rfoot_pos_y() = F(10, 10) * x.rfoot_pos_y() + rfoot_offset.y();
      x_next.rfoot_pos_z() = F(11, 11) * x.rfoot_pos_z() + rfoot_offset.z();

      return x_next;
    }

    void updateJacobians(State &x, const Control &u)
    {

    }

    Eigen::Matrix<double, 12, 3> B;

    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    Eigen::Vector3d position_offset;
    Eigen::Vector3d lfoot_offset;
    Eigen::Vector3d rfoot_offset;

};