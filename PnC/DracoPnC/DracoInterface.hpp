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

    //These would be the new API functionality that we would want for integration into ADE, such that we can build a
    //system to navigate to a relative point on the ADE side of things.

    //We have two potential ways that we would like to direct the robot to walk in a simpler manner (both being
    //implemented if possible)
    //Walk() would simply walk forward forever until receiving a Halt() (described below)
    void Walk();

    //Walk(int) [or double, or whatever fits best] would walk the robot straight forward for a given distance
    //That distance could be a standard number of footsteps, or a physical distance in any unit, as long as we are able
    //to convert the distance to a point measured by ADE to this method's parameters
    void Walk(int distance);

    //TurnTo would direct the robot to turn in a direction
    //heading represents a 2 dimensional vector that the robot should face in the direction of.
    // - This could also be a point struct of some kind.
    void TurnTo(std::vector<double> heading);

    //Halt would send an interrupt command to stop planning and reset the robot to a safe home/waiting position
    //We would like to be able to cancel a goal sent to the robot through ADE by having a way for the robot to
    //halt it's motion. Currently you can send a zero step command to Walk, which will have the robot take a half step
    //and then stop, but it cannot interrupt as there is no message queuing or interruption in the DracoInterface as of
    //yet.
    void Halt();

    void GetCoMTrajectory(std::vector<Eigen::VectorXd>& com_des_list);
    void GetContactSequence(std::vector<Eigen::Isometry3d>& foot_target_list);
    bool IsTrajectoryUpdated();
};
