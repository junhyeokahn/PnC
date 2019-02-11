#include <PnC/CartPolePnC/CtrlSet/BasicCtrl.hpp>
#include <PnC/CartPolePnC/CartPoleInterface.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>
#include <Configuration.h>

BasicCtrl::BasicCtrl(RobotSystem* _robot) : Controller(_robot) {
    myUtils::pretty_constructor(2, "Basic Ctrl");

    ctrl_count_ = 0;
    duration_ = 100000;
}

BasicCtrl::~BasicCtrl(){
}

void BasicCtrl::oneStep(void* _cmd){
    ((CartPoleCommand*)_cmd)->jtrq = 0.;
    ++ctrl_count_;
}

void BasicCtrl::firstVisit(){
}

void BasicCtrl::lastVisit(){
}

bool BasicCtrl::endOfPhase(){
    if(ctrl_count_ * SERVO_RATE > duration_){
        return true;
    }
    return false;
}

void BasicCtrl::ctrlInitialization(const YAML::Node& node){
}
