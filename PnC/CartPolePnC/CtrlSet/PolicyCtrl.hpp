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
    void setModelPath(std::string model_path);

   protected:
    double duration_;
    int ctrl_count_;

    NeuralNetModel* nn_policy_;
    NeuralNetModel* nn_valfn_;
};
