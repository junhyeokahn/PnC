#pragma once

#include "PnC/AtlasPnC/AtlasDefinition.hpp"
#include "PnC/EnvInterface.hpp"
#include "PnC/Test.hpp"

class AtlasStateEstimator;
class AtlasStateProvider;

class AtlasSensorData {
   public:
    AtlasSensorData() {
        q = Eigen::VectorXd::Zero(Atlas::n_adof);
        qdot = Eigen::VectorXd::Zero(Atlas::n_adof);
        imu_ang_vel = Eigen::VectorXd::Zero(3);
        imu_acc = Eigen::VectorXd::Zero(3);
        rfoot_contact = false;
        lfoot_contact = false;
    }
    virtual ~AtlasSensorData() {}

    Eigen::VectorXd q;
    Eigen::VectorXd qdot;
    Eigen::VectorXd imu_ang_vel;
    Eigen::VectorXd imu_acc;
    bool rfoot_contact;
    bool lfoot_contact;
};

class AtlasCommand {
   public:
    AtlasCommand() {
        q = Eigen::VectorXd::Zero(Atlas::n_adof);
        qdot = Eigen::VectorXd::Zero(Atlas::n_adof);
        jtrq = Eigen::VectorXd::Zero(Atlas::n_adof);
    }
    virtual ~AtlasCommand() {}

    Eigen::VectorXd q;
    Eigen::VectorXd qdot;
    Eigen::VectorXd jtrq;
};

class AtlasInterface : public EnvInterface {
   protected:
    void _ParameterSetting();

    AtlasStateEstimator* state_estimator_;
    AtlasStateProvider* sp_;

    void CropTorque_(AtlasCommand*);
    bool Initialization_(AtlasSensorData*, AtlasCommand*);
    void SetStopCommand_(AtlasSensorData*, AtlasCommand*);

    int count_;
    int waiting_count_;
    Eigen::VectorXd cmd_jpos_;
    Eigen::VectorXd cmd_jvel_;
    Eigen::VectorXd cmd_jtrq_;

   public:
    AtlasInterface();
    virtual ~AtlasInterface();
    virtual void getCommand(void* _sensor_data, void* _command_data);

    Eigen::Isometry3d GetTargetIso();
};
