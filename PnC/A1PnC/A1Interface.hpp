#pragma once

#include "PnC/EnvInterface.hpp"
//#include "PnC/Test.hpp"
#include "PnC/A1PnC/A1Definition.hpp"


class A1StateProvider;
class A1StateEstimator;

class A1SensorData{
    public:
      A1SensorData(){

        imu_ang_vel = Eigen::VectorXd::Zero(3);
        imu_acc = Eigen::VectorXd::Zero(3);

        q = Eigen::VectorXd::Zero(12);
        qdot = Eigen::VectorXd::Zero(12);
        jtrq = Eigen::VectorXd::Zero(12);
        virtual_q = Eigen::VectorXd::Zero(6);
        virtual_qdot = Eigen::VectorXd::Zero(6);

        flfoot_contact = false;
        frfoot_contact = false;
        rlfoot_contact = false;
        rrfoot_contact = false;

      }
      virtual ~A1SensorData() {}

      Eigen::VectorXd imu_ang_vel;
      Eigen::VectorXd imu_acc;
      Eigen::VectorXd q;
      Eigen::VectorXd qdot;
      Eigen::VectorXd jtrq;
      Eigen::VectorXd virtual_q;
      Eigen::VectorXd virtual_qdot;

      //int flf_wrench;
      //int frf_wrench;
      //int rlf_wrench;
      //int rrf_wrench;
      // Eigen::VectorXd flf_wrench;
      // Eigen::VectorXd frf_wrench;
      // Eigen::VectorXd rlf_wrench;
      // Eigen::VectorXd rrf_wrench;

      bool flfoot_contact;
      bool frfoot_contact;
      bool rlfoot_contact;
      bool rrfoot_contact;
};


class A1Command{
    public:
      A1Command(){
          q = Eigen::VectorXd::Zero(12);
          qdot = Eigen::VectorXd::Zero(12);
          jtrq = Eigen::VectorXd::Zero(12);
      }
      virtual ~A1Command(){}

      Eigen::VectorXd q;
      Eigen::VectorXd qdot;
      Eigen::VectorXd jtrq;
};



class A1Interface : public EnvInterface{
    protected:
      int waiting_count_;

      void _ParameterSetting();
      bool _Initialization(A1SensorData*, A1Command*);
      bool _UpdateTestCommand(A1Command* test_cmd);
      void _SetStopCommand(A1SensorData*, A1Command* cmd);
      void _CopyCommand(A1Command* cmd);

      A1StateProvider* sp_;
      A1StateEstimator* state_estimator_;

      Eigen::VectorXd cmd_jpos_;
      Eigen::VectorXd cmd_jvel_;
      Eigen::VectorXd cmd_jtrq_;
      Eigen::VectorXd data_torque_;

      // safety
      Eigen::VectorXd jpos_max_;
      Eigen::VectorXd jpos_min_;
      Eigen::VectorXd jvel_max_;
      Eigen::VectorXd jvel_min_;
      Eigen::VectorXd jtrq_max_;
      Eigen::VectorXd jtrq_min_;
      bool stop_test_;

    public:
      A1Interface();
      virtual ~A1Interface();
      virtual void getCommand(void* _sensor_data, void* _command_data);

};





