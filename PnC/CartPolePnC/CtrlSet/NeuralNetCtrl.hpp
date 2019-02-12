#pragma once

#include <PnC/Controller.hpp>
#include <zmq.hpp>
#include <PnC/NeuralNetwork/NeuralNetModel.hpp>

class CartPoleCommand;
class RobotSystem;

class NeuralNetCtrl: public Controller{
    public:
        NeuralNetCtrl(RobotSystem* _robot);
        virtual ~NeuralNetCtrl();

        virtual void oneStep(void* _cmd);
        virtual void firstVisit();
        virtual void lastVisit();
        virtual bool endOfPhase();
        virtual void ctrlInitialization(const YAML::Node& node);

        void setDuration(double time) { duration_ = time; }
    protected:
        double duration_;
        int ctrl_count_;
        int timesteps_per_actorbatch_;

        zmq::context_t *context_;
        zmq::socket_t *data_socket_;
        zmq::socket_t *policy_valfn_socket_;

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

        double alive_bonus_;
        double pole_cost_;
        double cart_cost_;
        double quad_input_cost_;
};
