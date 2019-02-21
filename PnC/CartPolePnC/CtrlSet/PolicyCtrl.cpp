#include <Configuration.h>
#include <PnC/CartPolePnC/CartPoleDefinition.hpp>
#include <PnC/CartPolePnC/CartPoleInterface.hpp>
#include <PnC/CartPolePnC/CtrlSet/PolicyCtrl.hpp>
#include <PnC/NeuralNetwork/NeuralNetModel.hpp>
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
    nn_policy_->GetOutput(obs, action_lower_bound_, action_upper_bound_,
            output, mean, neglogp);
    Eigen::MatrixXd val = nn_valfn_->GetOutput(obs);

    ((CartPoleCommand*)_cmd)->jtrq = output(0, 0);
    ((CartPoleCommand*)_cmd)->jtrq_mean = mean(0, 0);
    ((CartPoleCommand*)_cmd)->neglogp = neglogp(0);

    // scale the action for actual robot
    ((CartPoleCommand*)_cmd)->jtrq *= action_scale_;

    ++ctrl_count_;

    // !! TEST !! //
    //std::cout << "# ========================================================= #" << std::endl;
    //myUtils::pretty_print(obs, std::cout, "observation");
    //std::cout << "mean : " << mean(0, 0) << std::endl;
    //std::cout << "value : " << val(0, 0) << std::endl;
    //std::cout << "neglogp : " << neglogp(0) << std::endl;
    //std::cout << "output : " << output(0, 0) << std::endl;
    //if (ctrl_count_ == 5) {
        //exit(0);
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
    std::string model_folder;
    myUtils::readParameter(node, "model_path", model_folder);
    std::string model_dir =
        THIS_COM "RLData/CartPole/" + model_folder + "/model.yaml";
    YAML::Node model_cfg = YAML::LoadFile(model_dir);
    nn_policy_ = new NeuralNetModel(model_cfg["pol_params"], true);
    nn_valfn_ = new NeuralNetModel(model_cfg["valfn_params"], false);
    nn_policy_->PrintInfo();
    nn_valfn_->PrintInfo();
}
