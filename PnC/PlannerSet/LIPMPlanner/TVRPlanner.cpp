#include <Configuration.h>
#include <Eigen/Eigenvalues>
#include <PnC/PlannerSet/LIPMPlanner/TVRPlanner.hpp>
#include <Utils/IO/IOUtilities.hpp>

TVRPlanner::TVRPlanner()
    : com_vel_limit_(2), b_set_omega_(false), planner_save_data_(11) {
    t_prime_.resize(2);
    kappa_.resize(2);
    x_step_length_limit_.resize(2);
    y_step_length_limit_.resize(2);
    com_vel_limit_.resize(2);
    R_w_t_ = Eigen::MatrixXd::Identity(2, 2);
}

TVRPlanner::~TVRPlanner() {}

void TVRPlanner::_computeSwitchingState(
    double swing_time, const Eigen::Vector3d& com_pos,
    const Eigen::Vector3d& com_vel, const Eigen::Vector3d& stance_foot_loc,
    std::vector<Eigen::Vector2d>& switching_state) {
    double A, B;
    for (int i(0); i < 2; ++i) {
        A = ((com_pos[i] - stance_foot_loc[i]) + com_vel[i] / omega_) / 2.;
        B = ((com_pos[i] - stance_foot_loc[i]) - com_vel[i] / omega_) / 2.;
        switching_state[i][0] = A * exp(omega_ * swing_time) +
                                B * exp(-omega_ * swing_time) +
                                stance_foot_loc[i];
        switching_state[i][1] = omega_ * (A * exp(omega_ * swing_time) -
                                          B * exp(-omega_ * swing_time));
    }
}

// global CoM pos
void TVRPlanner::getNextFootLocation(const Eigen::Vector3d& com_pos,
                                                const Eigen::Vector3d& com_vel,
                                                Eigen::Vector3d& target_loc,
                                                const void* additional_input,
                                                void* additional_output) {
    if (!b_set_omega_) {
        printf("[Reversal Planner] Omega is not set\n");
        exit(0);
    }
    TVRParameter* _input = ((TVRParameter*)additional_input);
    TVROutput* _output = ((TVROutput*)additional_output);
    _UpdateRotation(_input->yaw_angle);

    std::vector<Eigen::Vector2d> switch_state(2);
    _computeSwitchingState(_input->swing_time, com_pos, com_vel,
                           _input->stance_foot_loc, switch_state);

    // (x, y, xdot, ydot)
    _output->switching_state[0] = switch_state[0][0];
    _output->switching_state[1] = switch_state[1][0];
    _output->switching_state[2] = switch_state[0][1];
    _output->switching_state[3] = switch_state[1][1];
    // printf("switching position: %f, %f\n", switch_state[0][0],
    // switch_state[1][0]);
    // printf("switching velocity: %f, %f\n", switch_state[0][1],
    // switch_state[1][1]);

    // !! TEST !!
    int check_switch(_check_switch_velocity(switch_state));
    // int
    // check_switch(_check_switch_velocity_considering_rotation(switch_state));
    // !! TEST !!
    double new_swing_time(_input->swing_time);
    int count(0);
    while (check_switch != 0) {
        if (check_switch > 0) {  // Too small velocity increase time
            new_swing_time *= 1.1;
            myUtils::color_print(myColor::BoldRed,
                                 "Too small velocity.. increase swing time: " +
                                     std::to_string(new_swing_time),
                                 true);
        } else {  // Too larget velocity decrease time
            new_swing_time *= 0.9;
            myUtils::color_print(myColor::BoldRed,
                                 "Too small velocity.. decrease swing time: " +
                                     std::to_string(new_swing_time),
                                 true);
        }
        _computeSwitchingState(new_swing_time, com_pos, com_vel,
                               _input->stance_foot_loc, switch_state);

        // !! TEST !!
        check_switch = _check_switch_velocity(switch_state);
        // check_switch =
        // _check_switch_velocity_considering_rotation(switch_state);
        // !! TEST !!
        ++count;
        if (count > 0) break;
    }
    _output->time_modification = new_swing_time - _input->swing_time;

    for (int i(0); i < 2; ++i) {
        double exp_weight =
            (exp(omega_ * t_prime_[i]) + exp(-omega_ * t_prime_[i])) /
            (exp(omega_ * t_prime_[i]) - exp(-omega_ * t_prime_[i]));

        target_loc[i] = switch_state[i][0] +
                        (switch_state[i][1] / omega_) * exp_weight +
                        kappa_[i] * (_input->des_loc[i] - switch_state[i][0]);
    }
    target_loc[2] = 0.;

    // _StepLengthCheck(target_loc, switch_state);
    // !! TEST !!
    _StepLengthCheck(target_loc, _input->b_positive_sidestep,
                     _input->stance_foot_loc);
    //_StepLengthCheckConsideringRotation(target_loc,
    //_input->b_positive_sidestep, _input->stance_foot_loc);
    // !! TEST !!

    // save data
    for (int i(0); i < 2; ++i) {
        planner_save_data_[i] = com_pos[i];
        planner_save_data_[2 + i] = com_vel[i];
        planner_save_data_[4 + i] = switch_state[i][0];
        planner_save_data_[6 + i] = switch_state[i][1];
        planner_save_data_[8 + i] = target_loc[i];
    }
    planner_save_data_[10] = new_swing_time;
    myUtils::saveVector(planner_save_data_, "planner_data");
}

int TVRPlanner::_check_switch_velocity(
    const std::vector<Eigen::Vector2d>& switch_state) {
    int ret(0);
    double x_vel(switch_state[0][1]);
    double y_vel(switch_state[1][1]);

    // X
    if (x_vel > 0.) {
        if (x_vel < com_vel_limit_[0]) ret = 1;
        if (x_vel > com_vel_limit_[1]) ret = -1;
    } else {
        if (x_vel > -com_vel_limit_[0]) ret = 1;
        if (x_vel < -com_vel_limit_[1]) ret = -1;
    }

    // Y
    if (y_vel > 0.) {
        if (y_vel < com_vel_limit_[0]) ret = 1;
        if (y_vel > com_vel_limit_[1]) ret = -1;
    } else {
        if (y_vel > -com_vel_limit_[0]) ret = 1;
        if (y_vel < -com_vel_limit_[1]) ret = -1;
    }

    return ret;
}

int TVRPlanner::_check_switch_velocity_considering_rotation(
    const std::vector<Eigen::Vector2d>& switch_state) {
    int ret(0);
    Eigen::VectorXd global_com_vel = Eigen::VectorXd::Zero(2);
    global_com_vel << switch_state[0][1], switch_state[1][1];
    Eigen::VectorXd local_com_vel = Eigen::VectorXd::Zero(2);
    local_com_vel = R_w_t_.transpose() * global_com_vel;

    // X
    double x_vel(local_com_vel[0]);
    double y_vel(local_com_vel[1]);
    if (x_vel > 0.) {
        if (x_vel < com_vel_limit_[0]) ret = 1;
        if (x_vel > com_vel_limit_[1]) ret = -1;
    } else {
        if (x_vel > -com_vel_limit_[0]) ret = 1;
        if (x_vel < -com_vel_limit_[1]) ret = -1;
    }

    // Y
    if (y_vel > 0.) {
        if (y_vel < com_vel_limit_[0]) ret = 1;
        if (y_vel > com_vel_limit_[1]) ret = -1;
    } else {
        if (y_vel > -com_vel_limit_[0]) ret = 1;
        if (y_vel < -com_vel_limit_[1]) ret = -1;
    }
    return ret;
}

void TVRPlanner::_StepLengthCheck(
    Eigen::Vector3d& target_loc, bool b_positive_sidestep,
    const Eigen::Vector3d& stance_foot) {
    // X limit check
    double x_step_length(target_loc[0] - stance_foot[0]);
    if (x_step_length < x_step_length_limit_[0]) {
        target_loc[0] = stance_foot[0] + x_step_length_limit_[0];
        myUtils::color_print(
            myColor::BoldRed,
            "x step length hit min: " + std::to_string(x_step_length), true);
        myUtils::color_print(myColor::BoldRed,
                             "new x step: (" + std::to_string(target_loc[0]) +
                                 ", " + std::to_string(stance_foot[0]) + ")",
                             true);
    }
    if (x_step_length > x_step_length_limit_[1]) {
        target_loc[0] = stance_foot[0] + x_step_length_limit_[1];
        myUtils::color_print(
            myColor::BoldRed,
            "x step length hit max: " + std::to_string(x_step_length), true);
        myUtils::color_print(myColor::BoldRed,
                             "new x step: (" + std::to_string(target_loc[0]) +
                                 ", " + std::to_string(stance_foot[0]) + ")",
                             true);
    }

    // Y limit check
    double y_step_length(target_loc[1] - stance_foot[1]);
    if (b_positive_sidestep) {  // move to left
        if (y_step_length < y_step_length_limit_[0]) {
            target_loc[1] = stance_foot[1] + y_step_length_limit_[0];
            myUtils::color_print(
                myColor::BoldRed,
                "y step length hit min: " + std::to_string(y_step_length),
                true);
            myUtils::color_print(myColor::BoldRed,
                                 "new y step: (" +
                                     std::to_string(target_loc[1]) + ", " +
                                     std::to_string(stance_foot[1]) + ")",
                                 true);
        }

        if (y_step_length > y_step_length_limit_[1]) {
            target_loc[1] = stance_foot[1] + y_step_length_limit_[1];
            myUtils::color_print(
                myColor::BoldRed,
                "y step length hit max: " + std::to_string(y_step_length),
                true);
            myUtils::color_print(myColor::BoldRed,
                                 "new y step: (" +
                                     std::to_string(target_loc[1]) + ", " +
                                     std::to_string(stance_foot[1]) + ")",
                                 true);
        }

    } else {  // move to right
        if (-y_step_length < y_step_length_limit_[0]) {
            target_loc[1] = stance_foot[1] - y_step_length_limit_[0];
            myUtils::color_print(
                myColor::BoldRed,
                "y step length hit min: " + std::to_string(y_step_length),
                true);
            myUtils::color_print(myColor::BoldRed,
                                 "new y step: (" +
                                     std::to_string(target_loc[1]) + ", " +
                                     std::to_string(stance_foot[1]) + ")",
                                 true);
        }

        if (-y_step_length > y_step_length_limit_[1]) {
            target_loc[1] = stance_foot[1] - y_step_length_limit_[1];
            myUtils::color_print(
                myColor::BoldRed,
                "y step length hit max: " + std::to_string(y_step_length),
                true);
            myUtils::color_print(myColor::BoldRed,
                                 "new y step: (" +
                                     std::to_string(target_loc[1]) + ", " +
                                     std::to_string(stance_foot[1]) + ")",
                                 true);
        }
    }
}

void TVRPlanner::_StepLengthCheck(
    Eigen::Vector3d& target_loc,
    const std::vector<Eigen::Vector2d>& switch_state) {
    // X limit check
    double x_step_length(target_loc[0] - switch_state[0][0]);
    if (x_step_length < x_step_length_limit_[0]) {
        target_loc[0] = switch_state[0][0] + x_step_length_limit_[0];
        printf("x step length hit min: %f\n", x_step_length);
        printf("new x step: %f, %f \n", target_loc[0], switch_state[0][0]);
    }
    if (x_step_length > x_step_length_limit_[1]) {
        target_loc[0] = switch_state[0][0] + x_step_length_limit_[1];
        printf("x step length hit max: %f\n", x_step_length);
        printf("new x step: %f, %f \n", target_loc[0], switch_state[0][0]);
    }

    // Y limit check
    double y_step_length(target_loc[1] - switch_state[1][0]);
    if (switch_state[1][1] > 0) {  // move to left
        if (y_step_length < y_step_length_limit_[0]) {
            target_loc[1] = switch_state[1][0] + y_step_length_limit_[0];
        }

        if (y_step_length > y_step_length_limit_[1]) {
            target_loc[1] = switch_state[1][0] + y_step_length_limit_[1];
        }

    } else {  // move to right
        if (-y_step_length < y_step_length_limit_[0])
            target_loc[1] = switch_state[1][0] - y_step_length_limit_[0];
        if (-y_step_length > y_step_length_limit_[1])
            target_loc[1] = switch_state[1][0] - y_step_length_limit_[1];
    }
}

void TVRPlanner::PlannerInitialization(const YAML::Node& node) {
    try {
        Eigen::VectorXd tmp;
        myUtils::readParameter(node, "t_prime", tmp);
        for (int i = 0; i < 2; ++i) {
            t_prime_[i] = tmp[i];
        }
        myUtils::readParameter(node, "kappa", tmp);
        for (int i = 0; i < 2; ++i) {
            kappa_[i] = tmp[i];
        }
        myUtils::readParameter(node, "x_step_length_limit", tmp);
        for (int i = 0; i < 2; ++i) {
            x_step_length_limit_[i] = tmp[i];
        }
        myUtils::readParameter(node, "y_step_length_limit", tmp);
        for (int i = 0; i < 2; ++i) {
            y_step_length_limit_[i] = tmp[i];
        }
        myUtils::readParameter(node, "com_velocity_limit", tmp);
        for (int i = 0; i < 2; ++i) {
            com_vel_limit_[i] = tmp[i];
        }
    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
    }
}

void TVRPlanner::CheckEigenValues(double swing_time) {
    Eigen::MatrixXd A(2, 2);
    printf("omega, swing_time: %f, %f\n", omega_, swing_time);

    for (int i(0); i < 2; ++i) {
        double coth = cosh(omega_ * t_prime_[i]) / sinh(omega_ * t_prime_[i]);

        A(0, 0) = 1 - kappa_[i] + kappa_[i] * cosh(omega_ * swing_time);
        A(0, 1) = (sinh(omega_ * swing_time) +
                   (1. - cosh(omega_ * swing_time)) * coth) /
                  omega_;
        A(1, 0) = kappa_[i] * omega_ * sinh(omega_ * swing_time);
        A(1, 1) = cosh(omega_ * swing_time) - sinh(omega_ * swing_time) * coth;

        Eigen::VectorXcd eivals = A.eigenvalues();
        printf("%d - axis eigen value:\n", i);
        std::cout << eivals << std::endl;
    }
}

void TVRPlanner::_UpdateRotation(double yaw_angle) {
    R_w_t_(0, 0) = cos(yaw_angle);
    R_w_t_(1, 0) = sin(yaw_angle);
    R_w_t_(1, 0) = -sin(yaw_angle);
    R_w_t_(1, 1) = cos(yaw_angle);
}

void TVRPlanner::_StepLengthCheckConsideringRotation(
    Eigen::Vector3d& target_loc, bool b_positive_sidestep,
    const Eigen::Vector3d& stance_foot) {
    Eigen::Vector2d stance_foot_in_torso, target_loc_in_torso,
        v_stance_target_in_torso;
    stance_foot_in_torso << stance_foot[0], stance_foot[1];
    stance_foot_in_torso = R_w_t_.transpose() * stance_foot_in_torso;
    target_loc_in_torso << target_loc[0], target_loc[1];
    target_loc_in_torso = R_w_t_.transpose() * target_loc_in_torso;
    v_stance_target_in_torso = target_loc_in_torso - stance_foot_in_torso;

    // X limit check
    if (v_stance_target_in_torso[0] < x_step_length_limit_[0]) {
        target_loc_in_torso[0] =
            stance_foot_in_torso[0] + x_step_length_limit_[0];
        myUtils::color_print(myColor::BoldRed,
                             "x step length hit min: " +
                                 std::to_string(v_stance_target_in_torso[0]),
                             true);
        // myUtils::color_print(myColor::BoldRed, "new x step: (" +
        // std::to_string(target_loc[0]) + ", " + std::to_string(stance_foot[0])
        // + ")", true);
    }
    if (v_stance_target_in_torso[0] > x_step_length_limit_[1]) {
        target_loc_in_torso[0] =
            stance_foot_in_torso[0] + x_step_length_limit_[1];
        myUtils::color_print(myColor::BoldRed,
                             "x step length hit max: " +
                                 std::to_string(v_stance_target_in_torso[0]),
                             true);
        // myUtils::color_print(myColor::BoldRed, "new x step: ("+
        // std::to_string(target_loc[0]) + ", " + std::to_string(stance_foot[0])
        // + ")", true);
    }

    // Y limit check
    if (b_positive_sidestep) {  // move to left
        if (v_stance_target_in_torso[1] < y_step_length_limit_[0]) {
            target_loc_in_torso[1] =
                stance_foot_in_torso[1] + y_step_length_limit_[0];
            myUtils::color_print(
                myColor::BoldRed,
                "y step length hit min: " +
                    std::to_string(v_stance_target_in_torso[1]),
                true);
            // myUtils::color_print(myColor::BoldRed, "new y step: ("+
            // std::to_string(target_loc[1]) + ", " +
            // std::to_string(stance_foot[1]) + ")", true);
        }

        if (v_stance_target_in_torso[1] > y_step_length_limit_[1]) {
            target_loc_in_torso[1] =
                stance_foot_in_torso[1] + y_step_length_limit_[1];
            myUtils::color_print(
                myColor::BoldRed,
                "y step length hit max: " +
                    std::to_string(v_stance_target_in_torso[1]),
                true);
            // myUtils::color_print(myColor::BoldRed, "new y step: ("+
            // std::to_string(target_loc[1]) + ", " +
            // std::to_string(stance_foot[1]) + ")", true);
        }

    } else {  // move to right
        if (-v_stance_target_in_torso[1] < y_step_length_limit_[0]) {
            target_loc_in_torso[1] =
                stance_foot_in_torso[1] - y_step_length_limit_[0];
            myUtils::color_print(
                myColor::BoldRed,
                "y step length hit min: " +
                    std::to_string(v_stance_target_in_torso[1]),
                true);
            // myUtils::color_print(myColor::BoldRed, "new y step: ("+
            // std::to_string(target_loc[1]) + ", " +
            // std::to_string(stance_foot[1]) + ")", true);
        }

        if (-v_stance_target_in_torso[1] > y_step_length_limit_[1]) {
            target_loc_in_torso[1] =
                stance_foot_in_torso[1] - y_step_length_limit_[1];
            myUtils::color_print(
                myColor::BoldRed,
                "y step length hit max: " +
                    std::to_string(v_stance_target_in_torso[1]),
                true);
            // myUtils::color_print(myColor::BoldRed, "new y step: ("+
            // std::to_string(target_loc[1]) + ", " +
            // std::to_string(stance_foot[1]) + ")", true);
        }
    }
    target_loc[0] = (R_w_t_ * target_loc_in_torso)[0];
    target_loc[1] = (R_w_t_ * target_loc_in_torso)[1];
}
