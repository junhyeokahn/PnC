#include <PnC/MPC/CMPC.hpp>
#include <Utils/Math/minjerk_one_dim.hpp>

Eigen::Vector3d des_com_xy_pos_given_feet(const Eigen::MatrixXd & r_feet){
    Eigen::Vector3d rfoot_avg = 0.5*(r_feet.col(0) + r_feet.col(1));
    Eigen::Vector3d lfoot_avg = 0.5*(r_feet.col(2) + r_feet.col(3));
    return 0.5*(rfoot_avg + lfoot_avg);
}

int main(int argc, char ** argv){
    CMPC convex_mpc;
    // convex_mpc.simulate_toy_mpc();

    // System Params
    convex_mpc.setRobotMass(50);  // (kilograms)
    Eigen::MatrixXd I_robot_body =
        1.0 * Eigen::MatrixXd::Identity(3, 3);  // Body Inertia matrix
    // I_robot_body(0,0) = 07;
    // I_robot_body(1,1) = 0.26;
    // I_robot_body(2,2) = 0.242;
    convex_mpc.setRobotInertia(I_robot_body);

    // MPC Params
    int horizon_length = 15;
    convex_mpc.setHorizon(horizon_length);  // horizon timesteps
    convex_mpc.setDt(0.025);    // (seconds) per horizon
    convex_mpc.setMu(0.9);      //  friction coefficient
    convex_mpc.setMaxFz(500);   // (Newtons) maximum vertical reaction force per foot.

    // Starting robot state
    // Current reduced state of the robot
    // x = [Theta, p, omega, pdot, g] \in \mathbf{R}^13
    Eigen::VectorXd x0(13);

    double init_roll(0), init_pitch(0), init_yaw(0.0), init_com_x(0),
           init_com_y(0), init_com_z(0.75), init_roll_rate(0), init_pitch_rate(0),
           init_yaw_rate(0), init_com_x_rate(0), init_com_y_rate(0),
           init_com_z_rate(0);

    x0 = convex_mpc.getx0(init_roll, init_pitch, init_yaw, init_com_x, init_com_y,
                          init_com_z, init_roll_rate, init_pitch_rate, init_yaw_rate,
                          init_com_x_rate, init_com_y_rate, init_com_z_rate);

    // Feet Params
    double foot_length = 0.05;   // 5cm distance between toe and heel
    double nominal_width = 0.1;  // 10cm distance between left and right feet

    // Initial Feet Configuration
    // Set Foot contact locations w.r.t world
    Eigen::MatrixXd r_feet(3, 4);  // Each column is a reaction force in x,y,z
    r_feet.setZero();

    // Flat ground contact, z = 0.0
    // 4 Contact Configuration
    // Right Foot Front
    r_feet(0, 0) = foot_length / 2.0;     // x
    r_feet(1, 0) = -nominal_width / 2.0;  // y
    // Right Foot Back
    r_feet(0, 1) = -foot_length / 2.0;    // x
    r_feet(1, 1) = -nominal_width / 2.0;  // y
    // Left Foot Front
    r_feet(0, 2) = foot_length / 2.0;    // x
    r_feet(1, 2) = nominal_width / 2.0;  // y
    // Left Foot Back
    r_feet(0, 3) = -foot_length / 2.0;   // x
    r_feet(1, 3) = nominal_width / 2.0;  // y

    // Foot landing configuration
    Eigen::MatrixXd r_feet_land = r_feet;
    Eigen::Vector3d rfoot_translate(0.25, 0.0, 0.0);
    Eigen::Vector3d lfoot_translate(0.25, 0.0, 0.0);

    std::cout << "r_feet start location:" << std::endl;
    std::cout << r_feet << std::endl;

    r_feet_land.col(0) += rfoot_translate; r_feet_land.col(1) += rfoot_translate;
    r_feet_land.col(2) += lfoot_translate; r_feet_land.col(3) += lfoot_translate;

    std::cout << "r_feet landing location:" << std::endl;
    std::cout << r_feet_land << std::endl;


    // Set custom gait cycle
    double swing_time = 0.2;
    double transition_time = 0.2;
    double biped_walking_offset = swing_time + transition_time;
    double total_gait_duration = 2.0*swing_time + 2.0*transition_time;
    std::shared_ptr<GaitCycle> gait_weight_transfer(new GaitCycle(swing_time, total_gait_duration, {0.0, 0.0, biped_walking_offset, biped_walking_offset}));
    convex_mpc.setCustomGaitCycle(gait_weight_transfer);
    convex_mpc.setPreviewStartTime(0.0);

    // Set initial and terminal com pos and orientation
    Eigen::Vector3d ini_com_pos(init_com_x, init_com_y, init_com_z);
    Eigen::Vector3d ini_ori_pos(init_roll, init_pitch, init_yaw);
    Eigen::Vector3d des_end_com_pos = des_com_xy_pos_given_feet(r_feet_land);
    des_end_com_pos += Eigen::Vector3d(0.0, 0.0, init_com_z);
    Eigen::Vector3d des_end_ori(0.0, 0.0, 0.0); // roll, pitch, yaw

    // Initialize reference minimum jerk trajectory
    std::vector<MinJerk_OneDimension> com_min_jerk_ref;
    std::vector<MinJerk_OneDimension> ori_min_jerk_ref;

    double time_start = 0.0;
    for(int i = 0; i < 3; i++){
        com_min_jerk_ref.push_back(MinJerk_OneDimension(Eigen::Vector3d(ini_com_pos[i], 0.0, 0.0), 
                                                        Eigen::Vector3d(des_end_com_pos[i], 0.0, 0.0), 
                                                        time_start, 
                                                        total_gait_duration) );        
        ori_min_jerk_ref.push_back(MinJerk_OneDimension(Eigen::Vector3d(ini_ori_pos[i], 0.0, 0.0), 
                                                        Eigen::Vector3d(des_end_ori[i], 0.0, 0.0), 
                                                        time_start, 
                                                        total_gait_duration) ); 
    }

    // Test minimum jerk trajectory
    double test_time = 0.0;
    double test_time_dur = 1.0;
    double test_dt = 0.01;
    int n_test = static_cast<int>(test_time_dur/test_dt);
    printf("t, r, p, y, x, y, z, rx, ry, rz, dx, dy, dz\n");

    double com_x, com_y, com_z, rx, ry, rz;
    double dcom_x, dcom_y, dcom_z, drx, dry, drz;
    for(int i = 0; i < n_test; i++){
        // Get CoM value
        com_min_jerk_ref[0].getPos(test_time, com_x); com_min_jerk_ref[0].getVel(test_time, dcom_x);
        com_min_jerk_ref[1].getPos(test_time, com_y); com_min_jerk_ref[1].getVel(test_time, dcom_y);
        com_min_jerk_ref[2].getPos(test_time, com_y); com_min_jerk_ref[2].getVel(test_time, dcom_z);

        // Get Ori value
        ori_min_jerk_ref[0].getPos(test_time, rx); ori_min_jerk_ref[0].getVel(test_time, drx);
        ori_min_jerk_ref[1].getPos(test_time, ry); ori_min_jerk_ref[1].getVel(test_time, dry);
        ori_min_jerk_ref[2].getPos(test_time, ry); ori_min_jerk_ref[2].getVel(test_time, drz);

        printf("%0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f\n",
               test_time, rx, ry, rz, com_x, com_y, com_z, rx, ry, rz, dcom_x, dcom_y, dcom_z);
        // Increment test time
        test_time += test_dt;
    }


    // int n = convex_mpc->getStateVecDim(); // This is always size 13.
    // Eigen::VectorXd mpc_Xref = Eigen::VectorXd::Zero(n*mpc_horizon_); // Create the desired state vector evolution




}