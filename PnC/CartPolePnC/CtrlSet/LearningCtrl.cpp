#include <rl_msg.pb.h>
#include <PnC/CartPolePnC/CartPoleDefinition.hpp>
#include <PnC/CartPolePnC/CartPoleInterface.hpp>
#include <PnC/CartPolePnC/CtrlSet/LearningCtrl.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/IO/ZmqUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>

LearningCtrl::LearningCtrl(RobotSystem* _robot, int mpi_idx, int env_idx)
    : Controller(_robot) {
    myUtils::pretty_constructor(2, "Learning Ctrl");

    duration_ = 100000;
    ctrl_count_ = 0;

    // =========================================================================
    // Construct zmq socket and connect
    // =========================================================================
    std::string ip_sub_pub, ip_req_rep;
    YAML::Node yaml_cfg =
        YAML::LoadFile(THIS_COM "Config/CartPole/TEST/RL_TEST.yaml");
    myUtils::readParameter(
        yaml_cfg["control_configuration"]["learning_ctrl"]["protocol"],
        "ip_sub_pub_prefix", ip_sub_pub);
    myUtils::readParameter(
        yaml_cfg["control_configuration"]["learning_ctrl"]["protocol"],
        "ip_req_rep_prefix", ip_req_rep);

    context_ = new zmq::context_t(1);
    data_socket_ = new zmq::socket_t(*context_, ZMQ_PUB);
    policy_valfn_socket_ = new zmq::socket_t(*context_, ZMQ_REP);
    ip_sub_pub += std::to_string(mpi_idx) + std::to_string(env_idx);
    ip_req_rep += std::to_string(mpi_idx) + std::to_string(env_idx);
    data_socket_->bind(ip_sub_pub);
    policy_valfn_socket_->bind(ip_req_rep);
    myUtils::PairAndSync(*data_socket_, *policy_valfn_socket_, 1);

    // =========================================================================
    // Build neural net policy
    // =========================================================================
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

    // =========================================================================
    // Build neural net value function
    // =========================================================================
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
}

LearningCtrl::~LearningCtrl() {
    delete context_;
    delete data_socket_;
    delete policy_valfn_socket_;
    delete nn_policy_;
    delete nn_valfn_;
}

void LearningCtrl::oneStep(void* _cmd) {
    Eigen::MatrixXd obs(1, nn_policy_->GetNumInput());
    obs << robot_->getQ()[0], robot_->getQ()[1], robot_->getQdot()[0],
        robot_->getQdot()[1];

    Eigen::MatrixXd output;
    Eigen::MatrixXd mean;
    Eigen::VectorXd neglogp;
    nn_policy_->GetOutput(obs, output, mean, neglogp);

    ((CartPoleCommand*)_cmd)->jtrq = myUtils::cropValue(
        output(0, 0), action_lower_bound_[0], action_upper_bound_[0], "jtrq");
    ((CartPoleCommand*)_cmd)->jtrq_mean = mean(0, 0);
    ((CartPoleCommand*)_cmd)->neglogp = neglogp(0);

    SendRLData_(obs, (CartPoleCommand*)_cmd);

    // scale the action for actual robot
    ((CartPoleCommand*)_cmd)->jtrq *= action_scale_;

    ++ctrl_count_;
}

void LearningCtrl::SendRLData_(Eigen::MatrixXd obs, CartPoleCommand* cmd) {
    // =========================================================================
    // Serialize dataset via protobuf
    // =========================================================================
    RL::DataSet pb_data_set;

    // set count
    pb_data_set.set_count(ctrl_count_);

    // set done
    bool done(false);
    if ((obs(0, 0) < terminate_obs_lower_bound_[0]) ||
        (obs(0, 1) > terminate_obs_upper_bound_[0]) ||
        (obs(0, 2) < terminate_obs_lower_bound_[1]) ||
        (obs(0, 3) > terminate_obs_upper_bound_[1])) {
        done = true;
    }
    pb_data_set.set_done(done);

    // set reward : alive bonus - pole angle - cart pos - torque usage
    float reward(0.0);
    if (!done) {
        reward += alive_reward_;
    }
    reward -= cart_reward_ * std::abs(obs(0, 0));
    reward -= pole_reward_ * std::abs(obs(0, 1));
    reward -= quad_input_reward_ * cmd->jtrq * cmd->jtrq;
    reward *= reward_scale_;
    pb_data_set.set_reward(reward);

    // set observation, (cart pos, pole angle, cart vel, pole ang vel) \in R^{4}
    for (int i = 0; i < 2; ++i) {
        pb_data_set.add_observation(obs(0, i));
    }
    for (int i = 0; i < 2; ++i) {
        pb_data_set.add_observation(obs(0, i + 2));
    }

    // set action
    pb_data_set.add_action(cmd->jtrq);

    // set action
    pb_data_set.add_action_mean(cmd->jtrq_mean);

    // set neglogpacs
    pb_data_set.set_neglogp(cmd->neglogp);

    // set vpred
    pb_data_set.set_value((nn_valfn_->GetOutput(obs))(0, 0));

    std::string pb_data_set_serialized;
    pb_data_set.SerializeToString(&pb_data_set_serialized);

    // =========================================================================
    // Send message via zmq
    // =========================================================================
    zmq::message_t zmq_msg(pb_data_set_serialized.size());
    memcpy((void*)zmq_msg.data(), pb_data_set_serialized.c_str(),
           pb_data_set_serialized.size());
    data_socket_->send(zmq_msg);
}

void LearningCtrl::firstVisit() {}

void LearningCtrl::lastVisit() {}

bool LearningCtrl::endOfPhase() {
    if (ctrl_count_ * CartPoleAux::ServoRate > duration_) {
        return true;
    }
    return false;
}

void LearningCtrl::ctrlInitialization(const YAML::Node& node) {
    try {
        myUtils::readParameter(node["reward"], "alive_reward", alive_reward_);
        myUtils::readParameter(node["reward"], "pole_reward", pole_reward_);
        myUtils::readParameter(node["reward"], "cart_reward", cart_reward_);
        myUtils::readParameter(node["reward"], "quad_input_reward",
                               quad_input_reward_);

    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
        exit(0);
    }
}
