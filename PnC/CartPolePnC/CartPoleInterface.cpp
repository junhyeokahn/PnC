#include <stdio.h>
#include <chrono>
#include <thread>

#include <PnC/CartPolePnC/CartPoleInterface.hpp>
#include <Configuration.h>
#include <Utils/IO/ZmqUtilities.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <cart_pole_msg.pb.h>

CartPoleInterface::CartPoleInterface() : Interface()
{
    // =========================================================================
    // Construct zmq socket and connect
    // =========================================================================
    context_ = new zmq::context_t(1);
    data_socket_ = new zmq::socket_t(*context_, ZMQ_PUB);
    policy_socket_ = new zmq::socket_t(*context_, ZMQ_REP);
    data_socket_->bind(IP_RL_SUB_PUB);
    policy_socket_->bind(IP_RL_REQ_REP);
    myUtils::PairAndSync(*data_socket_, *policy_socket_, 1);

    // =========================================================================
    // Build neural net policy
    // =========================================================================
    zmq::message_t zmq_msg;
    CartPole::StochasticPolicyParam pb_policy_param;
    std::cout << "receiving policy data" << std::endl;
    policy_socket_->recv(&zmq_msg);
    myUtils::StringSend(*policy_socket_, "");
    pb_policy_param.ParseFromArray(zmq_msg.data(), zmq_msg.size());
    layers_.clear();
    for (int idx_layer = 0; idx_layer < pb_policy_param.layers_size(); ++idx_layer) {
        int num_input(pb_policy_param.layers(idx_layer).num_input());
        int num_output(pb_policy_param.layers(idx_layer).num_output());
        ActivationFunction act_fn(static_cast<ActivationFunction>(pb_policy_param.layers(idx_layer).act_fn()));
        Eigen::MatrixXd weight = Eigen::MatrixXd::Zero(num_input, num_output);
        Eigen::MatrixXd bias = Eigen::MatrixXd::Zero(1, num_output);

        for (int row_idx = 0; row_idx < num_input; ++row_idx) {
            for (int col_idx = 0; col_idx < num_output; ++col_idx) {
                weight(row_idx, col_idx) = pb_policy_param.layers(idx_layer).weight(row_idx * num_output + col_idx);
            }
        }
        for (int idx_bias = 0; idx_bias < num_output; ++idx_bias) {
            bias(0, idx_bias) = pb_policy_param.layers(idx_layer).bias(idx_bias);
        }
        layers_.push_back(Layer(weight, bias, act_fn));
    }
    nn_policy_ = new NeuralNetModel(layers_);

    // =========================================================================
    // Parse RL config
    // =========================================================================
    try {
        YAML::Node rl_cfg = YAML::LoadFile(THIS_COM"Config/CartPole/RL.yaml");
        myUtils::readParameter(rl_cfg["env"], "obs_lower_bound", obs_lower_bound_);
        myUtils::readParameter(rl_cfg["env"], "obs_upper_bound", obs_upper_bound_);
        myUtils::readParameter(rl_cfg["env"], "action_lower_bound", action_lower_bound_);
        myUtils::readParameter(rl_cfg["env"], "action_upper_bound", action_upper_bound_);
    } catch(std::runtime_error& e) {
        std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
    }
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

    // TODO
    //Eigen::VectorXd obs;
    //cmd->jtrq = (nn_policy_->GetOutPut(obs))[0];
    cmd->jtrq = 0.;

    SendRLDataSet_(data, cmd);

    ++count_;
}

void CartPoleInterface::SendRLDataSet_(CartPoleSensorData* data, CartPoleCommand* cmd) {
    // =========================================================================
    // Serialize dataset via protobuf
    // =========================================================================
    CartPole::DataSet pb_data_set;
    // set count
    pb_data_set.set_count(count_);
    // set done
    bool done(false);
    if ( (data->q[0] < obs_lower_bound_[0]) ||
         (data->q[0] > obs_upper_bound_[0]) ||
         (data->q[1] < obs_lower_bound_[2]) ||
         (data->q[1] > obs_upper_bound_[2]) )
    {
        done = true;
    }
    pb_data_set.set_done(done);
    // set reward
    float reward(0.0);
    if (!done) { reward = 1.0; }
    pb_data_set.set_reward(reward);
    // set observation, (jpos, jvel, jtrq) \in R^{5}
    for (int i = 0; i < data->q.size(); ++i) {
        pb_data_set.add_observation(data->q[i]);
    }
    for (int i = 0; i < data->qdot.size(); ++i) {
        pb_data_set.add_observation(data->qdot[i]);
    }

    std::string pb_data_set_serialized;
    pb_data_set.SerializeToString(&pb_data_set_serialized);

    // =========================================================================
    // Send message via zmq
    // =========================================================================
    zmq::message_t zmq_msg(pb_data_set_serialized.size());
    memcpy ((void *) zmq_msg.data(), pb_data_set_serialized.c_str(),
            pb_data_set_serialized.size());
    data_socket_->send(zmq_msg);
}
