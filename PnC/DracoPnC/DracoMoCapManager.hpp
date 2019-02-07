#pragma once

#include <Utils/DartpThread.hpp>
#include <PnC/DracoPnC/StateEstimator/BodyEstimator.hpp>
#include <Filters/Basic/filters.hpp>

class DracoStateProvider;
class RobotSystem;

////////////////////////////////////////////
// Must to be synchronized with MoCap system
/////////////////////////////////////////////
#define MOCAP_DATA_PORT 51128
#define IP_ADDRESS "192.168.0.144"
#define NUM_MARKERS 13

typedef struct{
    int visible[NUM_MARKERS];
    double data[NUM_MARKERS*3];
}draco_message;

enum MocapLed {
    cTorsoLed= 0,
    lTorsoLed= 1,
    rTorsoLed= 2
};

class DracoMoCapManager: public DartpThread{
    public:
        friend class BodyEstimator;

        DracoMoCapManager(RobotSystem* );
        virtual ~DracoMoCapManager();

        virtual void run(void);

        Eigen::Quaternion<double>  body_quat_; // R_global(mocap)_torso
        void CoordinateUpdateCall(){ b_update_call_ = true; }

    protected:
        std::vector<Eigen::Vector3d> healthy_led_list_;

        std::vector<filter*> body_led0_filter_;
        std::vector<filter*> body_led1_filter_;
        std::vector<filter*> body_led2_filter_;

        double initialization_duration_;
        Eigen::Vector3d offset_;
        Eigen::Quaternion<double> imu_body_ori_;

        DracoStateProvider * sp_;
        Eigen::MatrixXd R_coord_;
        bool b_update_call_;

        void _Print_message(const draco_message & msg);
        void _UpdateLEDPosData(const draco_message & msg);
        void _CoordinateUpdate(draco_message & msg);
        void _CoordinateChange(draco_message & msg);
        // R_torso_global(mocap)
        Eigen::MatrixXd _GetOrientation(const Eigen::Vector3d &,
                const Eigen::Vector3d &, const Eigen::Vector3d &);

        int socket_;
        std::vector<int> marker_cond_;

        Eigen::VectorXd led_pos_data_;
        Eigen::VectorXd led_kin_data_;
        Eigen::VectorXd led_pos_raw_data_;

        int lfoot_idx;
        int rfoot_idx;

        RobotSystem* robot_;
};
