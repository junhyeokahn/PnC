#include <PnC/DracoPnC/DracoInterface.hpp>
#include <ADE/DracoWrapper.hpp>
#include <thread>
#include <chrono>



DracoWrapper::DracoWrapper() : running_(false) {
    interface_ = new DracoInterface();
    simulator_ = new DracoSim(interface_);
}

void DracoWrapper::Initialize() {
    simulator_->StartSim();
    running_ = true;
}

void DracoWrapper::SetWalkRawCommand(double ft_length, double r_ft_width, double l_ft_width,
                                  double ori_inc, int num_step) {
    if (!running_) {
        throw std::bad_function_call();
    }
    interface_->Walk(ft_length, r_ft_width, l_ft_width, ori_inc, num_step);
    while(!interface_->IsReadyForNextCommand()) {
        std::this_thread::sleep_for (std::chrono::milliseconds (100));
    }
}

void DracoWrapper::SetWalkXCommand(double x) {
    if (!running_) {
        throw std::bad_function_call();
    }
    interface_->WalkInX(x);
    while(!interface_->IsReadyForNextCommand()) {
        std::this_thread::sleep_for (std::chrono::milliseconds (100));
    }
}

void DracoWrapper::SetWalkYCommand(double y) {
    if (!running_) {
        throw std::bad_function_call();
    }
    interface_->WalkInY(y);
    while(!interface_->IsReadyForNextCommand()) {
        std::this_thread::sleep_for (std::chrono::milliseconds (100));
    }
}

void DracoWrapper::SetTurnCommand( double th) {
    if (!running_) {
        throw std::bad_function_call();
    }
    interface_->Turn(th);
    while(!interface_->IsReadyForNextCommand()) {
        std::this_thread::sleep_for (std::chrono::milliseconds (100));
    }
}

void DracoWrapper::SetWalkToRelativeCommand(double x, double y, double th) {
    if (!running_) {
        throw std::bad_function_call();
    }
    interface_->WalkToRelativePositionAndOrientation(x, y, th);
    while(!interface_->IsReadyForNextCommand()) {
        std::this_thread::sleep_for (std::chrono::milliseconds (100));
    }
}

void DracoWrapper::SetHaltCommand() {
    myUtils::color_print(myColor::BoldRed,
                         "[[Halting]]");
}

//TODO
void DracoWrapper::SetMoveEndEffectorCommand(double x, double y, double z, double qw, double qx, double qy, double qz){
    myUtils::color_print(myColor::BoldRed,
                         "[[Moving End Effector: ( " + std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) + ")\n ["
                         + std::to_string(qw) + ", " + std::to_string(qx) + ", " + std::to_string(qy) + ", " + std::to_string(qz) + "] ]]");
}
void DracoWrapper::SetCloseGripperCommand(){
    myUtils::color_print(myColor::BoldRed,
                         "[[Closing Gripper]]");
}
void DracoWrapper::SetOpenGripperCommand(){
    myUtils::color_print(myColor::BoldRed,
                         "[[Opening Gripper]]");
}

void DracoWrapper::Shutdown() {
    simulator_->StopSim();
}


