#include <stdio.h>
#include <chrono>
#include <thread>

#include <PnC/CartPolePnC/CartPoleInterface.hpp>
#include <Configuration.h>
#include <Utils/IO/ZmqUtilities.hpp>
#include <rl_msg.pb.h>

CartPoleInterface::CartPoleInterface() : Interface()
{
    context_ = new zmq::context_t(1);
    data_socket_ = new zmq::socket_t(*context_, ZMQ_PUB);
    policy_socket_ = new zmq::socket_t(*context_, ZMQ_REP);
    // Construct zmq publisher
    data_socket_->bind(IP_RL_SUB_PUB);
    policy_socket_->bind(IP_RL_REQ_REP);
    myUtils::PairAndSync(*data_socket_, *policy_socket_, 1);

    // build neural net policy
    // TODO double check Constrution, Output of NN model
    zmq::message_t zmq_msg;
    RL::PolicyParam pb_policy_param;
    policy_socket_->recv(&zmq_msg);
    pb_policy_param.ParseFromArray(zmq_msg.data(), zmq_msg.size());
    layers_.clear();
    for (int idx_layer = 0; idx_layer < pb_policy_param.layers_size(); ++idx_layer) {
        int num_input(pb_policy_param.layers(idx_layer).num_input());
        int num_output(pb_policy_param.layers(idx_layer).num_output());
        ActivationFunction act_fn(static_cast<ActivationFunction>(pb_policy_param.layers(idx_layer).act_fn()));
        Eigen::MatrixXd weight(num_input, num_output);
        Eigen::VectorXd bias(num_output);
        for (int idx_weight = 0; idx_weight < num_input * num_output; ++idx_weight) {
            weight << pb_policy_param.layers(idx_layer).weight(idx_weight);
        }
        for (int idx_bias = 0; idx_bias < num_output; ++idx_bias) {
            bias << pb_policy_param.layers(idx_layer).bias(idx_bias);
        }
        layers_.push_back(Layer(weight, bias, act_fn));
    }
    nn_policy_ = new NeuralNetModel(layers_);
}

CartPoleInterface::~CartPoleInterface() {
    delete context_;
    delete data_socket_;
    delete policy_socket_;
    delete nn_policy_;
}

void CartPoleInterface::getCommand( void* _data, void* _cmd ) {

    CartPoleCommand* cmd = ((CartPoleCommand*) _cmd);
    CartPoleSensorData* data = ((CartPoleSensorData*) _data);

    cmd->jtrq = 0.;

    SendRLDataSet_(data, cmd);

    ++count_;
}

void CartPoleInterface::SendRLDataSet_(CartPoleSensorData* data, CartPoleCommand* cmd) {
    RL::DataSet pb_data_set;
    // set count
    pb_data_set.set_count(count_);
    // set reward
    float reward(0.0);
    pb_data_set.set_reward(reward);
    // set joint position, velocity, torque
    for (int i = 0; i < data->q.size(); ++i) {
        pb_data_set.add_joint_position(data->q[i]);
    }
    for (int i = 0; i < data->qdot.size(); ++i) {
        pb_data_set.add_joint_velocity(data->qdot[i]);
    }
    pb_data_set.set_joint_torque(cmd->jtrq);

    std::string pb_data_set_serialized;
    pb_data_set.SerializeToString(&pb_data_set_serialized);

    zmq::message_t zmq_msg(pb_data_set_serialized.size());
    memcpy ((void *) zmq_msg.data(), pb_data_set_serialized.c_str(),
            pb_data_set_serialized.size());
    data_socket_->send(zmq_msg);
}
