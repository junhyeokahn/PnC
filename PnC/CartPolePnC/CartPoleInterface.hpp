#pragma once

#include <Eigen/Dense>
#include <zmq.hpp>

#include <PnC/Interface.hpp>
#include <PnC/ReinforcementLearning/NeuralNetModel.hpp>

class CartPoleSensorData
{
public:
    Eigen::VectorXd q;
    Eigen::VectorXd qdot;
};

class CartPoleCommand
{
public:
    double jtrq;
};

class CartPoleInterface: public Interface
{
public:
    CartPoleInterface();
    virtual ~CartPoleInterface ();
    virtual void getCommand(void * _sensor_data, void * _command_data);

private:
    zmq::context_t *context_;
    zmq::socket_t *data_socket_;
    zmq::socket_t *policy_socket_;

    std::vector<Layer> layers_;
    NeuralNetModel* nn_policy_;

    void SendRLDataSet_(CartPoleSensorData* data, CartPoleCommand* cmd);
};
