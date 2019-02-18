#pragma once

#include <PnC/Controller.hpp>

class NeuralNetModel;
class RobotSystem;

class PolicyCtrl : public Controller {
   public:
    PolicyCtrl(RobotSystem* _robot);
    virtual ~PolicyCtrl();

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

    NeuralNetModel* nn_policy_;
    NeuralNetModel* nn_valfn_;

    Eigen::VectorXd obs_lower_bound_;
    Eigen::VectorXd obs_upper_bound_;
    Eigen::VectorXd terminate_obs_lower_bound_;
    Eigen::VectorXd terminate_obs_upper_bound_;
    Eigen::VectorXd action_lower_bound_;
    Eigen::VectorXd action_upper_bound_;

    double action_scale_;
};
