#include <Eigen/Dense>
#include <PnC/RobotSystem/RobotSystem.hpp>
#include <PnC/A1PnC/A1StateProvider.hpp>
#include <PnC/A1PnC/A1StateEstimator.hpp>
#include <Utils/IO/IOUtilities.hpp>


class GaitScheduler{

  public:
    GaitScheduler(RobotSystem* robot);

    ~GaitScheduler(){};

    void step(double current_time);

    void reset();
    Eigen::Vector4d stance_duration;
    Eigen::Vector4d duty_factor;
    Eigen::Vector4d swing_duration;
    Eigen::Vector4i current_contact_state;
    Eigen::Vector4i leg_state;

  protected:
    void _ParameterSetting();


    RobotSystem* robot_;
    A1StateProvider* sp_;

    Eigen::Vector4d normalized_phase;
    Eigen::Vector4d initial_state_ratio_in_cycle;
    Eigen::Vector4d initial_leg_phase;

    double contact_detection_phase_threshold;

    // 3 - lost contact
    // 2 - early contact
    // 1 - stance
    // 0 - swing
    Eigen::Vector4i next_leg_state;
    Eigen::Vector4i desired_leg_state;
    Eigen::Vector4i initial_leg_state;
};
