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
    enum LEG {LEFT = 0, RIGHT};

    PoseMeasurementModel()
    {
      H.setZero();
      V.setIdentity();
      V = V * 0.001;
    }

    void initialize(const double &gravity)
    {
      // assign values to LTI matrix C
      H.block(0, 0, 3, 3) = I;
      H.block(0, 6, 3, 3) = -I;
      H.block(3, 0, 3, 3) = I;
      H.block(3, 9, 3, 3) = -I;

      this->gravity = gravity;
    }

    void packAccelerationInput(const Eigen::Matrix3d &rot_world_to_base,
                               const Eigen::Vector3d &accelerometer,
                               Control &u_n)
    {
      u_n.accel_measurement_x = rot_world_to_base.row(0) * accelerometer;
      u_n.accel_measurement_y = rot_world_to_base.row(1) * accelerometer;
      u_n.accel_measurement_z = rot_world_to_base.row(2) * accelerometer + gravity;
    }

    void update_position_from_lfoot(const Eigen::Vector3d& lfoot_wrt_world,
                                    PoseMeasurement &measurementToUpdate)
    {
      measurementToUpdate.base_pose_lfoot_x() = lfoot_wrt_world.x();
      measurementToUpdate.base_pose_lfoot_y() = lfoot_wrt_world.y();
      measurementToUpdate.base_pose_lfoot_z() = lfoot_wrt_world.z();
    }

    void update_leg_covariance(LEG leg, double& level)
    {
      switch (leg) {
        case LEFT:
          V.block(0, 0, 3, 3) = level * I;
          break;
        case RIGHT:
          V.block(3, 3, 3, 3) = level * I;
          break;
      }
    }


    void update_position_from_lfoot(const State& xhat,
                                    PoseMeasurement &measurementToUpdate)
    {
      measurementToUpdate.base_pose_lfoot_x() = xhat.base_pos_x() - xhat.lfoot_pos_x();
      measurementToUpdate.base_pose_lfoot_y() = xhat.base_pos_y() - xhat.lfoot_pos_y();
      measurementToUpdate.base_pose_lfoot_z() = xhat.base_pos_z() - xhat.lfoot_pos_z();
    }

    void update_position_from_rfoot(const Eigen::Vector3d& rfoot_wrt_world,
                                    PoseMeasurement &measurementToUpdate)
    {
      measurementToUpdate.base_pose_rfoot_x() = rfoot_wrt_world.x();
      measurementToUpdate.base_pose_rfoot_y() = rfoot_wrt_world.y();
      measurementToUpdate.base_pose_rfoot_z() = rfoot_wrt_world.z();
    }

    void update_position_from_rfoot(const State& xhat,
                                    PoseMeasurement &measurementToUpdate)
    {
      measurementToUpdate.base_pose_rfoot_x() = xhat.base_pos_x() - xhat.rfoot_pos_x();
      measurementToUpdate.base_pose_rfoot_y() = xhat.base_pos_y() - xhat.rfoot_pos_y();
      measurementToUpdate.base_pose_rfoot_z() = xhat.base_pos_z() - xhat.rfoot_pos_z();
    }

    PoseMeasurement h(const State& x) const
    {
      PoseMeasurement y_next;

      y_next.base_pose_lfoot_x() = H(0, 0) * x.base_pos_x() + H(0, 6) * x.lfoot_pos_x();
      y_next.base_pose_lfoot_y() = H(1, 1) * x.base_pos_y() + H(1, 7) * x.lfoot_pos_y();
      y_next.base_pose_lfoot_z() = H(2, 2) * x.base_pos_z() + H(2, 8) * x.lfoot_pos_z();
      y_next.base_pose_rfoot_x() = H(3, 0) * x.base_pos_x() + H(3, 9) * x.rfoot_pos_x();
      y_next.base_pose_rfoot_y() = H(4, 1) * x.base_pos_y() + H(4, 10) * x.rfoot_pos_y();
      y_next.base_pose_rfoot_z() = H(5, 2) * x.base_pos_z() + H(5, 11) * x.rfoot_pos_z();

      return y_next;
    }

    void updateJacobians( const State& x)
    {

    }

    double gravity;

    const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
};