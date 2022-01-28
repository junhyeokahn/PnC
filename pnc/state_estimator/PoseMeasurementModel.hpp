#pragma once

#include "third_party/kalman_filters/LinearizedMeasurementModel.hpp"
#include "FloatingBaseSystemModel.hpp"

class PoseMeasurement : public Kalman::Vector<double, 6>
{
public:
    KALMAN_VECTOR(PoseMeasurement, double, 6);

    double& base_pose_lfoot_x() { return (*this) [pose_from_lfoot_x]; }
    double& base_pose_lfoot_y() { return (*this) [pose_from_lfoot_y]; }
    double& base_pose_lfoot_z() { return (*this) [pose_from_lfoot_z]; }
    double& base_pose_rfoot_x() { return (*this) [pose_from_rfoot_x]; }
    double& base_pose_rfoot_y() { return (*this) [pose_from_rfoot_y]; }
    double& base_pose_rfoot_z() { return (*this) [pose_from_rfoot_z]; }

    static constexpr size_t pose_from_lfoot_x = 0;
    static constexpr size_t pose_from_lfoot_y = 1;
    static constexpr size_t pose_from_lfoot_z = 2;
    static constexpr size_t pose_from_rfoot_x = 3;
    static constexpr size_t pose_from_rfoot_y = 4;
    static constexpr size_t pose_from_rfoot_z = 5;
};

//template<typename Vector3, template<class> class CovarianceBase = Kalman::StandardBase>
class PoseMeasurementModel : public Kalman::LinearizedMeasurementModel<State, PoseMeasurement, Kalman::StandardBase>
{
public:

    PoseMeasurementModel()
    {
      C.setZero();
    }

    void initialize(const Eigen::Vector3d &gravity)
    {
      // assign values to LTI matrix C
      C.block(0, 0, 3, 3) = I;
      C.block(0, 6, 3, 3) = -I;
      C.block(3, 0, 3, 3) = I;
      C.block(3, 9, 3, 3) = -I;

      this->gravity = gravity(2);
    }

    void packAccelerationInput(const Eigen::Matrix3d &rot_world_to_base,
                               const Eigen::Vector3d &accelerometer,
                               Control &u_n)
    {
      u_n.accel_measurement_x = rot_world_to_base.row(0) * accelerometer;
      u_n.accel_measurement_y = rot_world_to_base.row(1) * accelerometer;
      u_n.accel_measurement_z = rot_world_to_base.row(2) * accelerometer + gravity;
    }

    PoseMeasurement h(const State& x) const
    {
      PoseMeasurement y_next;

      y_next.base_pose_lfoot_x() = C(0, 0) * x.base_pos_x() + C(0, 6) * x.lfoot_pos_x();
      y_next.base_pose_lfoot_y() = C(1, 1) * x.base_pos_y() + C(1, 7) * x.lfoot_pos_y();
      y_next.base_pose_lfoot_z() = C(2, 2) * x.base_pos_z() + C(2, 8) * x.lfoot_pos_z();
      y_next.base_pose_rfoot_x() = C(3, 0) * x.base_pos_x() + C(3, 9) * x.rfoot_pos_x();
      y_next.base_pose_rfoot_y() = C(4, 1) * x.base_pos_y() + C(4, 10) * x.rfoot_pos_y();
      y_next.base_pose_rfoot_z() = C(5, 2) * x.base_pos_z() + C(5, 11) * x.rfoot_pos_z();

      return y_next;
    }

    void updateJacobians( const State& x)
    {
      H = C; // TODO rename C -> H for clarity
    }

    Eigen::Matrix<double, 6, 12> C;
    double gravity;

    const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
};