#pragma once

#include <rl_msg.pb.h>
#include <ReinforcementLearning/RLInterface/NeuralNetModel.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/IO/ZmqUtilities.hpp>

class RLData {
   public:
    int count;
    bool done;
    float reward;
    float value;
    Eigen::VectorXd action;
    Eigen::VectorXd action_mean;
    float neglogp;
    Eigen::VectorXd observation;
    bool b_data_filled;
};

class RLInterface {
   public:
    static RLInterface* GetRLInterface();
    ~RLInterface();

    // ========================================================================
    // Initialize Sockets : Construct ZmQ context, socket and Neural Net
    //                      Policy and Value Function
    // ========================================================================
    void Initialize(YAML::Node cfg, int mpi_idx, int env_idx);

    // =========================================================================
    // Serialize and Send message
    // =========================================================================
    void SendData();

    RLData* GetRLData() { return rl_data_; };
    NeuralNetModel* GetPolicy() { return nn_policy_; };
    NeuralNetModel* GetValueFn() { return nn_valfn_; };

   private:
    RLInterface();
    RLData* rl_data_;

    zmq::context_t* context_;
    zmq::socket_t* data_socket_;
    zmq::socket_t* policy_valfn_socket_;
    std::string ip_sub_pub_prefix_;
    std::string ip_req_rep_prefix_;

    std::vector<Layer> layers_;
    NeuralNetModel* nn_policy_;
    NeuralNetModel* nn_valfn_;

    bool b_initialized;
};
