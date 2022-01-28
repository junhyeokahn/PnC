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

//template<typename Vector3, template<class> class CovarianceBase = Kalman::StandardBase>
class FloatingBaseSystemModel : public Kalman::LinearizedSystemModel<State, Control, Kalman::StandardBase>
{
public:
    FloatingBaseSystemModel()
    {
      A.setZero();
      B.setZero();
    }

    void initialize(const double &delta_t)
    {
      // assign non-zero elements to A and B matrices
      A.block(0, 0, 3, 3) = I;
      A.block(0, 3, 3, 3) = delta_t * I;
      A.block(3, 3, 3, 3) = I;
      A.block(6, 6, 3, 3) = I;
      A.block(9, 9, 3, 3) = I;
      B.block(0, 0, 3, 3) = 0.5 * delta_t * delta_t * I;
      B.block(3, 0, 3, 3) = delta_t * I;
    }

    State f(const State& x, const Control& u) const
    {
      State x_next;

      // We perform the state prediction taking advantage of sparsity in A and B
      x_next.base_pos_x() =  A(0,0) * x.base_pos_x() + A(0,3) * x.base_vel_x() + B(0,0) * u.accel_meas_x();
      x_next.base_pos_y() =  A(1,1) * x.base_pos_y() + A(1,4) * x.base_vel_y() + B(1,1) * u.accel_meas_y();
      x_next.base_pos_z() =  A(2,2) * x.base_pos_z() + A(2,5) * x.base_vel_z() + B(2,2) * u.accel_meas_z();
      x_next.base_vel_x() = A(3, 3) * x.base_vel_x() + B(3,0) * u.accel_meas_x();
      x_next.base_vel_y() = A(4, 4) * x.base_vel_y() + B(4,1) * u.accel_meas_y();
      x_next.base_vel_z() = A(5, 5) * x.base_vel_z() + B(5,2) * u.accel_meas_z();
      x_next.lfoot_pos_x() = A(6, 6) * x.lfoot_pos_x();
      x_next.lfoot_pos_y() = A(7, 7) * x.lfoot_pos_y();
      x_next.lfoot_pos_z() = A(8, 8) * x.lfoot_pos_z();
      x_next.rfoot_pos_x() = A(9, 9) * x.rfoot_pos_x();
      x_next.rfoot_pos_y() = A(10, 10) * x.rfoot_pos_y();
      x_next.rfoot_pos_z() = A(11, 11) * x.rfoot_pos_z();

      return x_next;
    }

    void updateJacobians(State &x, const Control &u)
    {
      F = A; // TODO rename A -> F
      W.setIdentity();  // TODO assign based on better noise model
    }

    Eigen::Matrix<double, 12, 12> A;
    Eigen::Matrix<double, 12, 3> B;

    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

};