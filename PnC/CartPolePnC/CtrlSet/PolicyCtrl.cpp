#include <PnC/CartPolePnC/CtrlSet/PolicyCtrl.hpp>
#include <PnC/CartPolePnC/CartPoleInterface.hpp>
#include <PnC/CartPolePnC/CartPoleDefinition.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>
#include <Configuration.h>
#include <PnC/NeuralNetwork/NeuralNetModel.hpp>

PolicyCtrl::PolicyCtrl(RobotSystem* _robot) : Controller(_robot) {
    myUtils::pretty_constructor(2, "Policy Ctrl");

    ctrl_count_ = 0;
    duration_ = 100000;
}

PolicyCtrl::~PolicyCtrl(){
    delete nn_policy_;
    delete nn_valfn_;
}

void PolicyCtrl::oneStep(void* _cmd){
    ((CartPoleCommand*)_cmd)->jtrq = 0.;
    ++ctrl_count_;
}

void PolicyCtrl::firstVisit(){
}

void PolicyCtrl::lastVisit(){
}

bool PolicyCtrl::endOfPhase(){
    if(ctrl_count_ * CartPoleAux::ServoRate > duration_){
        return true;
    }
    return false;
}

void PolicyCtrl::ctrlInitialization(const YAML::Node& node){
}

void PolicyCtrl::setModelPath(std::string model_path){
    YAML::Node cfg = YAML::LoadFile(model_path);
    nn_policy_ = new NeuralNetModel(cfg["pol_params"]);
    nn_valfn_ = new NeuralNetModel(cfg["valfn_params"]);
}
