#include <PnC/ConvexMPC/GaitScheduler.hpp>
#include <PnC/A1PnC/A1StateProvider.hpp>
#include <PnC/RobotSystem/RobotSystem.hpp>
#include <Utils/IO/IOUtilities.hpp>


GaitScheduler::GaitScheduler(RobotSystem* robot){
    robot_ = robot;
    // sp_ = A1StateProvider::getStateProvider(robot_);

    myUtils::pretty_constructor(1, "Gait Scheduler");
    _ParameterSetting();
    reset();
}

void GaitScheduler::_ParameterSetting(){
    stance_duration << 0.3, 0.3, 0.3, 0.3;
    duty_factor << 0.5, 0.5, 0.5, 0.5;
    // TROT:
    // 0 is a swing leg
    // 1 is a stance leg
    // 2 is an early contact
    // 3 is a lost contact
    initial_leg_state << 0, 1, 1, 0;
    swing_duration = stance_duration / duty_factor[0] - stance_duration;
    initial_leg_phase << 0, 0, 0, 0;

    contact_detection_phase_threshold = 0.1;

    for(int i=0; i<4; ++i){
        if(initial_leg_state[i] == 0) { // Swing State
            initial_state_ratio_in_cycle[i] = 1. - duty_factor[i];
            next_leg_state[i] = 1; // Stance State Next
        }
        else { // Stance State
            initial_state_ratio_in_cycle[i] = duty_factor[i];
            next_leg_state[i] = 0; // Swing State Next
        }
    }
}


void GaitScheduler::reset(){
    normalized_phase = Eigen::VectorXd::Zero(4);
    desired_leg_state = initial_leg_state;
    leg_state = initial_leg_state;
}


void GaitScheduler::step(double current_time){
    current_contact_state << 1, 1, 1, 1;
    // current_contact_state << sp_->b_flfoot_contact, sp_->b_frfoot_contact,
    //                          sp_->b_rlfoot_contact, sp_->b_rrfoot_contact;

    double full_cycle_period = 0.;
    double augmented_time = 0.;
    double phase_in_full_cycle = 0.;
    double ratio = 0.;
    for(int i=0; i<4; ++i){
        full_cycle_period = stance_duration[i] / duty_factor[i];
        augmented_time = current_time + initial_leg_phase[i] * full_cycle_period;

        phase_in_full_cycle = (std::fmod(augmented_time, full_cycle_period)) / full_cycle_period;
        ratio = initial_state_ratio_in_cycle[i];

        if(phase_in_full_cycle < ratio){
            desired_leg_state[i] = initial_leg_state[i];
            normalized_phase[i] = phase_in_full_cycle / ratio;
        } else {
            desired_leg_state[i] = next_leg_state[i];
            normalized_phase[i] = (phase_in_full_cycle / ratio) / (1 - ratio);
        }
        leg_state[i] = desired_leg_state[i];

        if(leg_state[i] == 0 && current_contact_state[i]){
            std::cout << "Early Touchdown detected" << std::endl;
            leg_state[i] = 2;
        }
        if(leg_state[i] == 1 && !(current_contact_state[i])){
            leg_state[i] = 3;
        }
    }
}
