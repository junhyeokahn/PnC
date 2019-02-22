#pragma once

#include <Eigen/Dense>

#include <PnC/EnvInterface.hpp>

class CartPoleSensorData {
   public:
    Eigen::VectorXd q;
    Eigen::VectorXd qdot;
};

class CartPoleCommand {
   public:
    double jtrq;
    bool done;
};

class CartPoleInterface : public EnvInterface {
   public:
    CartPoleInterface();
    CartPoleInterface(int mpi_idx, int env_idx);
    virtual ~CartPoleInterface();
    virtual void getCommand(void* _sensor_data, void* _command_data);

    void ParameterSetting_();

    int mpi_idx_;
    int env_idx_;
    bool b_learning;

   private:
};
