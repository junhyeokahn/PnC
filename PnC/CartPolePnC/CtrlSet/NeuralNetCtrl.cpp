#include <PnC/CartPolePnC/CtrlSet/NeuralNetCtrl.hpp>
#include <PnC/CartPolePnC/CartPoleInterface.hpp>
#include <PnC/CartPolePnC/CartPoleDefinition.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>
#include <Utils/IO/ZmqUtilities.hpp>
#include <cart_pole_msg.pb.h>

NeuralNetCtrl::NeuralNetCtrl(RobotSystem* _robot) : Controller(_robot) {
    myUtils::pretty_constructor(2, "NN Ctrl");

    duration_ = 100000;
    ctrl_count_ = 0;
    timesteps_per_actorbatch_ = 256;

    // =========================================================================
    // Construct zmq socket and connect
    // =========================================================================
    context_ = new zmq::context_t(1);
    data_socket_ = new zmq::socket_t(*context_, ZMQ_PUB);
    policy_valfn_socket_ = new zmq::socket_t(*context_, ZMQ_REP);
    data_socket_->bind(std::string(CartPoleAux::IpSubPub));
    policy_valfn_socket_->bind(std::string(CartPoleAux::IpReqRep));
    myUtils::PairAndSync(*data_socket_, *policy_valfn_socket_, 1);

    // =========================================================================
    // Build neural net policy
    // =========================================================================
    zmq::message_t zmq_msg;
    CartPole::NeuralNetworkParam pb_policy_param;
    //std::cout << "receiving policy data" << std::endl;
    policy_valfn_socket_->recv(&zmq_msg);
    myUtils::StringSend(*policy_valfn_socket_, "");
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
    if (pb_policy_param.stochastic()) {
        Eigen::MatrixXd logstd = Eigen::MatrixXd::Zero(1, pb_policy_param.logstd_size());
        for (int output_idx = 0; output_idx < pb_policy_param.logstd_size(); ++output_idx) {
            logstd(0, output_idx) = pb_policy_param.logstd(output_idx);
        }
        nn_policy_ = new NeuralNetModel(layers_, logstd);
    } else {
        nn_policy_ = new NeuralNetModel(layers_);
    }
    //// TEST with stochastic
    //nn_policy_ = new NeuralNetModel(layers_);
    //// TEST

    // =========================================================================
    // Build neural net value function
    // =========================================================================
    //zmq::message_t zmq_msg; //do i need another zmq_msg?
    CartPole::NeuralNetworkParam pb_valfn_param;
    //std::cout << "receiving value function data" << std::endl;
    policy_valfn_socket_->recv(&zmq_msg);
    myUtils::StringSend(*policy_valfn_socket_, "");
    pb_valfn_param.ParseFromArray(zmq_msg.data(), zmq_msg.size());
    layers_.clear();
    for (int idx_layer = 0; idx_layer < pb_valfn_param.layers_size(); ++idx_layer) {
        int num_input(pb_valfn_param.layers(idx_layer).num_input());
        int num_output(pb_valfn_param.layers(idx_layer).num_output());
        ActivationFunction act_fn(static_cast<ActivationFunction>(pb_valfn_param.layers(idx_layer).act_fn()));
        Eigen::MatrixXd weight = Eigen::MatrixXd::Zero(num_input, num_output);
        Eigen::MatrixXd bias = Eigen::MatrixXd::Zero(1, num_output);

        for (int row_idx = 0; row_idx < num_input; ++row_idx) {
            for (int col_idx = 0; col_idx < num_output; ++col_idx) {
                weight(row_idx, col_idx) = pb_valfn_param.layers(idx_layer).weight(row_idx * num_output + col_idx);
            }
        }
        for (int idx_bias = 0; idx_bias < num_output; ++idx_bias) {
            bias(0, idx_bias) = pb_valfn_param.layers(idx_layer).bias(idx_bias);
        }
        layers_.push_back(Layer(weight, bias, act_fn));
    }
    nn_valfn_ = new NeuralNetModel(layers_);
}

NeuralNetCtrl::~NeuralNetCtrl(){
    delete context_;
    delete data_socket_;
    delete policy_valfn_socket_;
    delete nn_policy_;
    delete nn_valfn_;
}

void NeuralNetCtrl::oneStep(void* _cmd){
    Eigen::MatrixXd obs(1, nn_policy_->GetNumInput());
    obs << robot_->getQ()[0], robot_->getQ()[1], robot_->getQdot()[0], robot_->getQdot()[1];
    ((CartPoleCommand*)_cmd)->jtrq = (nn_policy_->GetOutput(obs))(0, 0);
    //((CartPoleCommand*)_cmd)->jtrq = 0.;
    if (ctrl_count_ < timesteps_per_actorbatch_) {
        SendRLData_(obs, (CartPoleCommand*)_cmd);
    }
    ++ctrl_count_;
}

void NeuralNetCtrl::SendRLData_(Eigen::MatrixXd obs, CartPoleCommand* cmd){
    // =========================================================================
    // Serialize dataset via protobuf
    // =========================================================================
    CartPole::DataSet pb_data_set;

    // set count
    pb_data_set.set_count(ctrl_count_);

    // set done
    bool done(false);
    if ( (obs(0, 0) < terminate_obs_lower_bound_[0]) ||
         (obs(0, 1) > terminate_obs_upper_bound_[0]) ||
         (obs(0, 2) < terminate_obs_lower_bound_[1]) ||
         (obs(0, 3) > terminate_obs_upper_bound_[1]) )
    {
        done = true;
    }
    pb_data_set.set_done(done);

    // set reward : alive bonus - pole angle - cart pos - torque usage
    float reward(0.0);
    if (!done) { reward += alive_bonus_; }
    reward -= cart_cost_ * std::abs(obs(0, 0));
    reward -= pole_cost_ * std::abs(obs(0, 1));
    reward -= quad_input_cost_ * cmd->jtrq * cmd->jtrq;
    pb_data_set.set_reward(reward);

    // set observation, (jpos, jvel) \in R^{4}
    for (int i = 0; i < 2; ++i) {
        pb_data_set.add_observation(robot_->getQ()[i]);
    }
    for (int i = 0; i < 2; ++i) {
        pb_data_set.add_observation(robot_->getQdot()[i]);
    }

    // set action
    pb_data_set.set_action(cmd->jtrq);

    // set vpred
    pb_data_set.set_vpred((nn_valfn_->GetOutput(obs))(0, 0));

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

void NeuralNetCtrl::firstVisit(){
}

void NeuralNetCtrl::lastVisit(){
}

bool NeuralNetCtrl::endOfPhase(){
    if(ctrl_count_ * CartPoleAux::ServoRate > duration_){
        return true;
    }
    return false;
}

void NeuralNetCtrl::ctrlInitialization(const YAML::Node& node){
    try {
        myUtils::readParameter(node, "obs_lower_bound", obs_lower_bound_);
        myUtils::readParameter(node, "obs_upper_bound", obs_upper_bound_);
        myUtils::readParameter(node, "terminate_obs_lower_bound", terminate_obs_lower_bound_);
        myUtils::readParameter(node, "terminate_obs_upper_bound", terminate_obs_upper_bound_);
        myUtils::readParameter(node, "action_lower_bound", action_lower_bound_);
        myUtils::readParameter(node, "action_upper_bound", action_upper_bound_);
        myUtils::readParameter(node, "timesteps_per_actorbatch", timesteps_per_actorbatch_);
        myUtils::readParameter(node, "alive_bonus", alive_bonus_);
        myUtils::readParameter(node, "pole_cost", pole_cost_);
        myUtils::readParameter(node, "cart_cost", cart_cost_);
        myUtils::readParameter(node, "quad_input_cost", quad_input_cost_);
    } catch(std::runtime_error& e) {
        std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
        exit(0);
    }

}
