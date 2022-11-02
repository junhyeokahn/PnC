#pragma once

/**
 * MARG Filter for a MARG (Magnetic, Angular Rate, and Gravity) sensor
 *
 * Implementation details found in:
 * Madgwick, S., 2010. An efficient orientation filter for inertial and
 * inertial/magnetic sensor arrays. Report x-io and University of Bristol (UK),
 * 25, pp.113-118.
 *
 * Code from https://forums.parallax.com/uploads/attachments/41167/106661.pdf
 */

#include <math.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

// System constants
const float deltat = 0.00125f; // sampling period in seconds (shown as 800 Hz)
const float gyroMeasError = 3.14159265358979 * (5.0f / 180.0f); // gyroscope measurement error in rad/s (shown as 5 deg/s)
const float gyroMeasDrift = 3.14159265358979 * (0.2f / 180.0f); // gyroscope measurement error in rad/s/s (shown as 0.2f deg/s/s)
const float beta = sqrt(3.0f / 4.0f) * gyroMeasError; // compute beta
const float zeta = sqrt(3.0f / 4.0f) * gyroMeasDrift; // compute zeta

class MARGFilter {
public:
    MARGFilter();
    ~MARGFilter();

    void initialize(float q1, float q2, float q3, float q4);

    void filterUpdate(float w_x, float w_y, float w_z,
                                  float a_x, float a_y, float a_z);

    void filterUpdate(float w_x, float w_y, float w_z,
                      float a_x, float a_y, float a_z,
                      float m_x, float m_y, float m_z);

    Eigen::Matrix3d getBaseRotation(void);

    Eigen::Quaterniond getQuaternion(void);

private:

    // Global system variables
    float a_x, a_y, a_z; // accelerometer measurements
    float w_x, w_y, w_z; // gyroscope measurements in rad/s
    float m_x, m_y, m_z; // magnetometer measurements
    float SEq_1, SEq_2, SEq_3, SEq_4;     // estimated orientation quaternion elements
    float b_x, b_z;                       // reference direction of flux in earth frame
    float w_bx, w_by, w_bz;               // estimate gyroscope biases error

    Eigen::Quaterniond quat;              // estimated quaternion

};