#include <Configuration.h>
#include <PnC/CartPolePnC/CartPoleDefinition.hpp>
#include <PnC/CartPolePnC/CartPoleInterface.hpp>
#include <PnC/CartPolePnC/CtrlSet/PolicyCtrl.hpp>
#include <ReinforcementLearning/RLInterface/NeuralNetModel.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>

PolicyCtrl::PolicyCtrl(RobotSystem* _robot) : Controller(_robot) {
    myUtils::pretty_constructor(2, "Policy Ctrl");

    ctrl_count_ = 0;
    duration_ = 100000;
}

PolicyCtrl::~PolicyCtrl() {
    delete nn_policy_;
    delete nn_valfn_;
}

void PolicyCtrl::oneStep(void* _cmd) {
    Eigen::MatrixXd obs(1, nn_policy_->GetNumInput());
    obs << robot_->getQ()[0], robot_->getQ()[1], robot_->getQdot()[0],
        robot_->getQdot()[1];

    Eigen::MatrixXd output;
    Eigen::MatrixXd mean;
    Eigen::VectorXd neglogp;
    nn_policy_->GetOutput(obs, action_lower_bound_, action_upper_bound_, output,
                          mean, neglogp);
    Eigen::MatrixXd val = nn_valfn_->GetOutput(obs);

    ((CartPoleCommand*)_cmd)->jtrq = output(0, 0);

    bool done(false);
    if ((obs(0, 0) < terminate_obs_lower_bound_[0]) ||
        (obs(0, 0) > terminate_obs_upper_bound_[0]) ||
        (obs(0, 1) < terminate_obs_lower_bound_[1]) ||
        (obs(0, 1) > terminate_obs_upper_bound_[1])) {
        done = true;
    }
    ((CartPoleCommand*)_cmd)->done = done;  // !! this will be no effects !!

    // scale the action for actual robot
    ((CartPoleCommand*)_cmd)->jtrq *= action_scale_;

    ++ctrl_count_;

    // !! TEST !! //
    // std::cout << "#
    // =========================================================#"
    //<< std::endl;
    // myUtils::pretty_print(obs, std::cout, " observation ");
    // std::cout << "mean : " << mean(0, 0) << std::endl;
    // std::cout << "value : " << val(0, 0) << std::endl;
    // std::cout << "neglogp : " << neglogp(0) << std::endl;
    // std::cout << "output : " << output(0, 0) << std::endl;
    // if (ctrl_count_ == 5) {
    // exit(0);
    //}
    // !! TEST !! //
}

void PolicyCtrl::firstVisit() {}

void PolicyCtrl::lastVisit() {}

bool PolicyCtrl::endOfPhase() {
    if (ctrl_count_ * CartPoleAux::ServoRate > duration_) {
        return true;
    }
    return false;
}

void PolicyCtrl::ctrlInitialization(const YAML::Node& node) {
    std::string model_path;
    myUtils::readParameter(node, "model_path", model_path);
    std::string model_yaml = THIS_COM + model_path;
    YAML::Node model_cfg = YAML::LoadFile(model_yaml);
    nn_policy_ = new NeuralNetModel(model_cfg["pol_params"], true);
    nn_valfn_ = new NeuralNetModel(model_cfg["valfn_params"], false);
    // nn_policy_->PrintInfo();
    // nn_valfn_->PrintInfo();
}
