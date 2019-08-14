#pragma once

#include "PnC/EnvInterface.hpp"
#include "PnC/Test.hpp"
#include "PnC/ValkyriePnC/ValkyrieDefinition.hpp"

class ValkyrieStateProvider;
class ValkyrieStateEstimator;

class ValkyrieSensorData {
   public:
    ValkyrieSensorData() {
        q = Eigen::VectorXd::Zero(Valkyrie::n_adof);
        qdot = Eigen::VectorXd::Zero(Valkyrie::n_adof);
        virtual_q = Eigen::VectorXd::Zero(Valkyrie::n_vdof);
        virtual_qdot = Eigen::VectorXd::Zero(Valkyrie::n_vdof);
        rfoot_contact = false;
        lfoot_contact = false;
    }
    virtual ~ValkyrieSensorData() {}

    Eigen::VectorXd q;
    Eigen::VectorXd qdot;
    Eigen::VectorXd virtual_q;
    Eigen::VectorXd virtual_qdot;
    bool rfoot_contact;
    bool lfoot_contact;
};

class ValkyrieCommand {
   public:
    ValkyrieCommand() {
        q = Eigen::VectorXd::Zero(Valkyrie::n_adof);
        qdot = Eigen::VectorXd::Zero(Valkyrie::n_adof);
        jtrq = Eigen::VectorXd::Zero(Valkyrie::n_adof);
    }
    virtual ~ValkyrieCommand() {}

    Eigen::VectorXd q;
    Eigen::VectorXd qdot;
    Eigen::VectorXd jtrq;
};

class ValkyrieInterface : public EnvInterface {
   protected:
    void _ParameterSetting();

    ValkyrieStateEstimator* state_estimator_;
    ValkyrieStateProvider* sp_;

    void CropTorque_(ValkyrieCommand*);
    bool Initialization_(ValkyrieSensorData*, ValkyrieCommand*);

    int count_;
    int waiting_count_;
    Eigen::VectorXd cmd_jpos_;
    Eigen::VectorXd cmd_jvel_;
    Eigen::VectorXd cmd_jtrq_;

    double prev_planning_moment_;

   public:
    ValkyrieInterface();
    virtual ~ValkyrieInterface();
    virtual void getCommand(void* _sensor_data, void* _command_data);

    void GetCoMTrajectory(std::vector<Eigen::VectorXd>& com_des_list);
    void GetContactSequence(std::vector<Eigen::Isometry3d>& foot_target_list);
    bool IsTrajectoryUpdated();
};
