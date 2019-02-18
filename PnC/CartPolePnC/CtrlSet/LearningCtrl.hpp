#pragma once

#include <PnC/Controller.hpp>
#include <PnC/NeuralNetwork/NeuralNetModel.hpp>
#include <zmq.hpp>

class CartPoleCommand;
class RobotSystem;

class LearningCtrl : public Controller {
   public:
    LearningCtrl(RobotSystem* _robot, int mpi_idx, int env_idx);
    virtual ~LearningCtrl();

    virtual void oneStep(void* _cmd);
    virtual void firstVisit();
    virtual void lastVisit();
    virtual bool endOfPhase();
    virtual void ctrlInitialization(const YAML::Node& node);

    void setDuration(double time) { duration_ = time; }
    void setObsLowerBound(const Eigen::VectorXd& lb) { obs_lower_bound_ = lb; }
    void setObsUpperBound(const Eigen::VectorXd& ub) { obs_upper_bound_ = ub; }
    void setTerminateObsLowerBound(const Eigen::VectorXd& lb) {
        terminate_obs_lower_bound_ = lb;
    }
    void setTerminateObsUpperBound(const Eigen::VectorXd& ub) {
        terminate_obs_upper_bound_ = ub;
    }
    void setActLowerBound(const Eigen::VectorXd& lb) {
        action_lower_bound_ = lb;
    }
    void setActUpperBound(const Eigen::VectorXd& ub) {
        action_upper_bound_ = ub;
    }
    void setActScale(double scale) { action_scale_ = scale; }

   protected:
    double duration_;
    int ctrl_count_;

    zmq::context_t* context_;
    zmq::socket_t* data_socket_;
    zmq::socket_t* policy_valfn_socket_;
    std::string ip_sub_pub_prefix_;
    std::string ip_req_rep_prefix_;

    std::vector<Layer> layers_;
    NeuralNetModel* nn_policy_;
    NeuralNetModel* nn_valfn_;

    Eigen::VectorXd obs_lower_bound_;
    Eigen::VectorXd obs_upper_bound_;
    Eigen::VectorXd terminate_obs_lower_bound_;
    Eigen::VectorXd terminate_obs_upper_bound_;
    Eigen::VectorXd action_lower_bound_;
    Eigen::VectorXd action_upper_bound_;

    void SendRLData_(Eigen::MatrixXd obs, CartPoleCommand* cmd);

    double alive_reward_;
    double pole_reward_;
    double cart_reward_;
    double quad_input_reward_;
    double reward_scale_;
    double action_scale_;
};
