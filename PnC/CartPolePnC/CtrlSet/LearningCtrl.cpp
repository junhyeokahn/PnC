#include <rl_msg.pb.h>
#include <PnC/CartPolePnC/CartPoleDefinition.hpp>
#include <PnC/CartPolePnC/CartPoleInterface.hpp>
#include <PnC/CartPolePnC/CtrlSet/LearningCtrl.hpp>
#include <ReinforcementLearning/RLInterface/RLInterface.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>

LearningCtrl::LearningCtrl(RobotSystem* _robot) : Controller(_robot) {
    myUtils::pretty_constructor(2, "Learning Ctrl");

    duration_ = 100000;
    ctrl_count_ = 0;
}

LearningCtrl::~LearningCtrl() {}

void LearningCtrl::oneStep(void* _cmd) {
    // =========================================================================
    // Set Data and Send
    // =========================================================================

    // 1. count
    RLInterface::GetRLInterface()->GetRLData()->count = ctrl_count_;

    // 2. observation
    Eigen::MatrixXd obs(1, nn_policy_->GetNumInput());
    obs << robot_->getQ()[0], robot_->getQ()[1], robot_->getQdot()[0],
        robot_->getQdot()[1];
    Eigen::VectorXd obs_vec(nn_policy_->GetNumInput());
    obs_vec << robot_->getQ()[0], robot_->getQ()[1], robot_->getQdot()[0],
        robot_->getQdot()[1];
    RLInterface::GetRLInterface()->GetRLData()->observation = obs_vec;

    // 3. nn outputs : action, action_mean, neglogp, value
    Eigen::MatrixXd output, mean;
    Eigen::VectorXd neglogp;
    nn_policy_->GetOutput(obs, action_lower_bound_, action_upper_bound_, output,
                          mean, neglogp);
    int n_output(output.cols());
    Eigen::VectorXd output_vec = Eigen::VectorXd::Zero(n_output);
    Eigen::VectorXd mean_vec = Eigen::VectorXd::Zero(n_output);
    float neglogp_val;
    for (int i = 0; i < n_output; ++i) {
        output_vec(i) = output(0, i);
        mean_vec(i) = mean(0, i);
    }
    neglogp_val = neglogp(0);

    RLInterface::GetRLInterface()->GetRLData()->action = output_vec;
    RLInterface::GetRLInterface()->GetRLData()->action_mean = mean_vec;
    RLInterface::GetRLInterface()->GetRLData()->neglogp = neglogp_val;
    RLInterface::GetRLInterface()->GetRLData()->value =
        (nn_valfn_->GetOutput(obs))(0, 0);

    ((CartPoleCommand*)_cmd)->jtrq = output(0, 0);
    ((CartPoleCommand*)_cmd)->jtrq *= action_scale_;

    // 4. done
    bool done(false);
    if ((obs(0, 0) < terminate_obs_lower_bound_[0]) ||
        (obs(0, 0) > terminate_obs_upper_bound_[0]) ||
        (obs(0, 1) < terminate_obs_lower_bound_[1]) ||
        (obs(0, 1) > terminate_obs_upper_bound_[1])) {
        done = true;
    }
    RLInterface::GetRLInterface()->GetRLData()->done = done;
    ((CartPoleCommand*)_cmd)->done = done;  // !! this will be no effects !!

    // 5. reward
    float reward(0.0);
    if (!done) {
        reward += alive_reward_;
    }
    reward -= cart_reward_ * std::abs(obs(0, 0));
    reward -= pole_reward_ * std::abs(obs(0, 1));
    reward -= quad_input_reward_ * output(0, 0) * output(0, 0);
    reward *= reward_scale_;
    RLInterface::GetRLInterface()->GetRLData()->reward = reward;

    // 6. fill out flag
    RLInterface::GetRLInterface()->GetRLData()->b_data_filled = true;
    RLInterface::GetRLInterface()->SendData();

    ++ctrl_count_;
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
        myUtils::readParameter(node["reward"], "reward_scale", reward_scale_);

    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
        exit(0);
    }
}
