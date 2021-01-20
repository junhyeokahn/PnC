#pragma once

#include "PnC/EnvInterface.hpp"
//#include "PnC/Test.hpp"
#include "PnC/A1PnC/A1Definition.hpp"




class A1StateProvider;

class A1SensorData{
    public:
      A1SensorData(){

        imu_ang_vel = Eigen::VectorXd::Zero(3);
        imu_acc = Eigen::VectorXd::Zero(3);

        q = Eigen::VectorXd::Zero(12);
        qdot = Eigen::VectorXd::Zero(12);
        jtrq = Eigen::VectorXd::Zero(12);
        // q_act = Eigen::VectorXd::Zero(A1::n_adof);
        // qdot_act = Eigen::VectorXd::Zero(A1::n_adof);
        virtual_q = Eigen::VectorXd::Zero(A1::n_vdof);
        virtual_qdot = Eigen::VectorXd::Zero(A1::n_vdof);

        flf_wrench = Eigen::VectorXd::Zero(6);
        frf_wrench = Eigen::VectorXd::Zero(6);
        rlf_wrench = Eigen::VectorXd::Zero(6);
        rrf_wrench = Eigen::VectorXd::Zero(6);

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
      // Eigen::VectorXd q_act;
      // Eigen::VectorXd qdot_act;
      Eigen::VectorXd virtual_q;
      Eigen::VectorXd virtual_qdot;

      Eigen::VectorXd flf_wrench;
      Eigen::VectorXd frf_wrench;
      Eigen::VectorXd rlf_wrench;
      Eigen::VectorXd rrf_wrench;

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


/*
class A1Interface : public EnvInterface{
    protected:
      void _ParameterSetting();

      A1StateProvider* sp_;
      // A1StateEstimator* state_estimator_;

      bool Initialization_(A1SensorData*, A1Command*);

      int count_;
      int waiting_count;
      bool test_initialized;
      Eigen::VectorXd cmd_jpos_;
      Eigen::VectorXd cmd_jvel_;
      Eigen::VectorXd cmd_jtrq_;


    public:
      A1Interface();
      virtual ~A1Interface();
      virtual void getCommand(void* _sensor_data, void* _command_data);

};




*/
