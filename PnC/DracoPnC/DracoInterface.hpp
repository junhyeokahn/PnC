#pragma once

#include "PnC/EnvInterface.hpp"
#include "PnC/Test.hpp"
#include "PnC/DracoPnC/DracoDefinition.hpp"

class DracoStateProvider;
class DracoStateEstimator;

class DracoSensorData {
   public:
    DracoSensorData() {
        q = Eigen::VectorXd::Zero(Draco::n_adof);
        qdot = Eigen::VectorXd::Zero(Draco::n_adof);
        virtual_q = Eigen::VectorXd::Zero(Draco::n_vdof);
        virtual_qdot = Eigen::VectorXd::Zero(Draco::n_vdof);
        lf_wrench = Eigen::VectorXd::Zero(6);
        rf_wrench = Eigen::VectorXd::Zero(6);
        rfoot_contact = false;
        lfoot_contact = false;
    }
    virtual ~DracoSensorData() {}

    Eigen::VectorXd q;
    Eigen::VectorXd qdot;
    Eigen::VectorXd virtual_q;
    Eigen::VectorXd virtual_qdot;
    Eigen::VectorXd lf_wrench;
    Eigen::VectorXd rf_wrench;
    bool rfoot_contact;
    bool lfoot_contact;
};

class DracoCommand {
   public:
    DracoCommand() {
        q = Eigen::VectorXd::Zero(Draco::n_adof);
        qdot = Eigen::VectorXd::Zero(Draco::n_adof);
        jtrq = Eigen::VectorXd::Zero(Draco::n_adof);
    }
    virtual ~DracoCommand() {}

    Eigen::VectorXd q;
    Eigen::VectorXd qdot;
    Eigen::VectorXd jtrq;
};

class DracoInterface : public EnvInterface {
   protected:
    void _ParameterSetting();

    DracoStateEstimator* state_estimator_;
    DracoStateProvider* sp_;

    void CropTorque_(DracoCommand*);
    bool Initialization_(DracoSensorData*, DracoCommand*);

    int count_;
    int waiting_count_;
    Eigen::VectorXd cmd_jpos_;
    Eigen::VectorXd cmd_jvel_;
    Eigen::VectorXd cmd_jtrq_;

    double prev_planning_moment_;

   public:
    DracoInterface();
    virtual ~DracoInterface();
    virtual void getCommand(void* _sensor_data, void* _command_data);
    void Walk(double ft_length, double r_ft_width, double l_ft_width,
              double ori_inc, int num_step);
    void WalkInX(double x, double max_delta_x=0.05);
    void WalkInY(double y, double max_delta_y=0.03);
    void Turn(double th, double max_delta_th=0.05);
    void WalkToRelativePositionAndOrientation(double x, double y, double th,
                                              double max_delta_x=0.05,
                                              double max_delta_y=0.03,
                                              double max_delta_th=0.05);
    bool IsReadyForNextCommand();

    void GetCoMTrajectory(std::vector<Eigen::VectorXd>& com_des_list);
    void GetContactSequence(std::vector<Eigen::Isometry3d>& foot_target_list);
    bool IsTrajectoryUpdated();
};
