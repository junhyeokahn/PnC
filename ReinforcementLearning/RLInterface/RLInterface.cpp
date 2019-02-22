#include <stdio.h>
#include <ReinforcementLearning/RLInterface/RLInterface.hpp>

RLInterface* RLInterface::GetRLInterface() {
    static RLInterface rl_interface_;
    return &rl_interface_;
}

RLInterface::RLInterface() : b_initialized(false) { rl_data_ = new RLData(); }

RLInterface::~RLInterface() {
    delete nn_policy_;
    delete nn_valfn_;
    delete context_;
    delete data_socket_;
    delete policy_valfn_socket_;
    delete rl_data_;
}

void RLInterface::SendData() {
    assert(b_initialized);
    assert(rl_data_->b_data_filled);

    RL::Data pb_data;
    pb_data.set_count(rl_data_->count);
    pb_data.set_done(rl_data_->done);
    pb_data.set_reward(rl_data_->reward);
    pb_data.set_value(rl_data_->value);
    pb_data.set_neglogp(rl_data_->neglogp);
    pb_data.set_done(rl_data_->done);
    assert(rl_data_->action.size() == rl_data_->action_mean.size());
    int n_action(rl_data_->action.size());
    for (int i = 0; i < n_action; ++i) {
        pb_data.add_action(rl_data_->action(i));
        pb_data.add_action_mean(rl_data_->action_mean(i));
    }
    int n_obs(rl_data_->observation.size());
    for (int i = 0; i < n_obs; ++i) {
        pb_data.add_observation(rl_data_->observation(i));
    }

    std::string pb_data_set_serialized;
    pb_data.SerializeToString(&pb_data_set_serialized);

    zmq::message_t zmq_msg(pb_data_set_serialized.size());
    memcpy((void*)zmq_msg.data(), pb_data_set_serialized.c_str(),
           pb_data_set_serialized.size());
    data_socket_->send(zmq_msg);

    rl_data_->b_data_filled = false;
}

void RLInterface::Initialize(YAML::Node cfg, int mpi_idx, int env_idx) {
    assert(!b_initialized);
    std::string ip_sub_pub, ip_req_rep;
    try {
        myUtils::readParameter(cfg, "ip_sub_pub_prefix", ip_sub_pub);
        myUtils::readParameter(cfg, "ip_req_rep_prefix", ip_req_rep);
    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
    }
    context_ = new zmq::context_t(1);
    data_socket_ = new zmq::socket_t(*context_, ZMQ_PUB);
    policy_valfn_socket_ = new zmq::socket_t(*context_, ZMQ_REP);
    ip_sub_pub += std::to_string(mpi_idx) + std::to_string(env_idx);
    ip_req_rep += std::to_string(mpi_idx) + std::to_string(env_idx);
    data_socket_->bind(ip_sub_pub);
    policy_valfn_socket_->bind(ip_req_rep);
    myUtils::PairAndSync(*data_socket_, *policy_valfn_socket_, 1);

    // Build neural net policy
    zmq::message_t zmq_msg;
    RL::NeuralNetworkParam pb_policy_param;
    policy_valfn_socket_->recv(&zmq_msg);
    myUtils::StringSend(*policy_valfn_socket_, "");
    pb_policy_param.ParseFromArray(zmq_msg.data(), zmq_msg.size());
    layers_.clear();
    for (int idx_layer = 0; idx_layer < pb_policy_param.layers_size();
         ++idx_layer) {
        int num_input(pb_policy_param.layers(idx_layer).num_input());
        int num_output(pb_policy_param.layers(idx_layer).num_output());
        ActivationFunction act_fn(static_cast<ActivationFunction>(
            pb_policy_param.layers(idx_layer).act_fn()));
        Eigen::MatrixXd weight = Eigen::MatrixXd::Zero(num_input, num_output);
        Eigen::MatrixXd bias = Eigen::MatrixXd::Zero(1, num_output);

        for (int row_idx = 0; row_idx < num_input; ++row_idx) {
            for (int col_idx = 0; col_idx < num_output; ++col_idx) {
                weight(row_idx, col_idx) =
                    pb_policy_param.layers(idx_layer).weight(
                        row_idx * num_output + col_idx);
            }
        }
        for (int idx_bias = 0; idx_bias < num_output; ++idx_bias) {
            bias(0, idx_bias) =
                pb_policy_param.layers(idx_layer).bias(idx_bias);
        }
        layers_.push_back(Layer(weight, bias, act_fn));
    }
    Eigen::VectorXd logstd =
        Eigen::VectorXd::Zero(pb_policy_param.logstd_size());
    for (int output_idx = 0; output_idx < pb_policy_param.logstd_size();
         ++output_idx) {
        logstd(output_idx) = pb_policy_param.logstd(output_idx);
    }
    nn_policy_ = new NeuralNetModel(layers_, logstd);

    // Build neural net value function
    RL::NeuralNetworkParam pb_valfn_param;
    policy_valfn_socket_->recv(&zmq_msg);
    myUtils::StringSend(*policy_valfn_socket_, "");
    pb_valfn_param.ParseFromArray(zmq_msg.data(), zmq_msg.size());
    layers_.clear();
    for (int idx_layer = 0; idx_layer < pb_valfn_param.layers_size();
         ++idx_layer) {
        int num_input(pb_valfn_param.layers(idx_layer).num_input());
        int num_output(pb_valfn_param.layers(idx_layer).num_output());
        ActivationFunction act_fn(static_cast<ActivationFunction>(
            pb_valfn_param.layers(idx_layer).act_fn()));
        Eigen::MatrixXd weight = Eigen::MatrixXd::Zero(num_input, num_output);
        Eigen::MatrixXd bias = Eigen::MatrixXd::Zero(1, num_output);

        for (int row_idx = 0; row_idx < num_input; ++row_idx) {
            for (int col_idx = 0; col_idx < num_output; ++col_idx) {
                weight(row_idx, col_idx) =
                    pb_valfn_param.layers(idx_layer).weight(
                        row_idx * num_output + col_idx);
            }
        }
        for (int idx_bias = 0; idx_bias < num_output; ++idx_bias) {
            bias(0, idx_bias) = pb_valfn_param.layers(idx_layer).bias(idx_bias);
        }
        layers_.push_back(Layer(weight, bias, act_fn));
    }
    nn_valfn_ = new NeuralNetModel(layers_);

    b_initialized = true;
}
