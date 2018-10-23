#include <dart/dart.hpp>

#include <Configuration.h>
#include <Utils/Utilities.hpp>
#include <Utils/DataManager.hpp>
#include <PnC/DracoPnC/StateEstimator/BasicAccumulation.hpp>

BasicAccumulation::BasicAccumulation():filtered_acc_(3){
    global_ori_quat_.w() = 1.;
    global_ori_quat_.x() = 0.;
    global_ori_quat_.y() = 0.;
    global_ori_quat_.z() = 0.;
    global_ori_ypr_.setZero(0);

    cutoff_freq_ = 2.0* M_PI *1.0; // 1Hz // (2*pi*frequency) rads/s
    for(int i(0); i<3; ++i){
        filtered_acc_[i] = new digital_lp_filter(cutoff_freq_, SERVO_RATE);
    }
    global_ang_vel_.setZero();
}
void BasicAccumulation::estimatorInitialization(
        const std::vector<double> & acc,
        const std::vector<double> & ang_vel){

    for(int i(0); i<3; ++i){
        global_ang_vel_[i] = ang_vel[i];
        filtered_acc_[i]->input(acc[i]);
    }
    _InitIMUOrientationEstimateFromGravity();
}

void BasicAccumulation::setSensorData(const std::vector<double> & acc,
        const std::vector<double> & ang_vel){

    // Convert body omega into a delta quaternion ------------------------------
    Eigen::Vector3d body_omega; body_omega.setZero();
    for(size_t i = 0; i < 3; i++){
        body_omega[i] = ang_vel[i];
    }
    Eigen::Quaternion<double> delta_quat_body = dart::math::expToQuat(body_omega*SERVO_RATE);

    Eigen::MatrixXd R_global_to_imu = global_ori_quat_.normalized().toRotationMatrix();
    global_ang_vel_ = R_global_to_imu * body_omega;

    // Perform orientation update via integration
    global_ori_quat_ = global_ori_quat_ * delta_quat_body;
    global_ori_ypr_ = dart::math::matrixToEulerZYX(global_ori_quat_.normalized().toRotationMatrix());

}


void BasicAccumulation::_InitIMUOrientationEstimateFromGravity(){
    Eigen::Vector3d g_B; g_B.setZero();
    for(int i(0); i<3; ++i){
        // We expect a negative number if gravity is pointing opposite of
        // the IMU direction
        g_B[i] = filtered_acc_[i]->output();
    }
    // Test Vector  ////////////////////////
    //g_B[0] = 0.1; g_B[1] = 0.4; g_B[2] = 0.5;
    //g_B[0] = 0.0; g_B[1] = 0.0; g_B[2] = 1.0;
    //qd rot_quat;
    //dynacore::convert(1.0, -0.5, 0.1, rot_quat);
    //Eigen::Matrix3d rot_mt(rot_quat);
    //g_B = rot_mt * g_B;
    //////////////////////////////////////////////
    g_B.normalize();

    double theta_pitch = atan2(g_B[0], g_B[2]);

    Eigen::Vector3d g_B_xz = g_B;
    g_B_xz[1]=0.;
    g_B_xz.normalize();

    double inner = g_B_xz.transpose() * g_B;
    double theta_roll = acos(inner);
    if(g_B[1]>0) theta_roll *= (-1.);

    Eigen::Quaternion<double> quat_pitch(dart::math::eulerYZXToMatrix( Eigen::Vector3d(0., theta_pitch, 0.)));
    Eigen::Quaternion<double> quat_roll(dart::math::eulerYZXToMatrix( Eigen::Vector3d(0., 0., theta_roll)));
    Eigen::Quaternion<double> local2Glob = quat_pitch * quat_roll;
    global_ori_quat_ = local2Glob.inverse();
    global_ori_ypr_ = dart::math::matrixToEulerZYX(global_ori_quat_.normalized().toRotationMatrix());

    ///////////////////////////////////////////////////////////////////
    //Eigen::Matrix3d Global_ori(global_ori_quat_);
    //v3 check_vec = Global_ori * g_B;
    //dynacore::pretty_print(check_vec, std::cout, "check vec");
    //dynacore::pretty_print(g_B, std::cout, "g_B");
    //printf("pitch, roll: %f, %f\n", theta_pitch, theta_roll);
}
