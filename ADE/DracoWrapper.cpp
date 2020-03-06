#include <PnC/DracoPnC/DracoInterface.hpp>
#include <ADE/DracoWrapper.hpp>
#include <thread>
#include <chrono>



DracoWrapper::DracoWrapper() : running_(false) {
    interface_ = new DracoInterface();
    arm_interface_ = new ScorpioInterface();
    arm2_interface_ = new ScorpioInterface();
    simulator_ = new DracoSim();
}

void DracoWrapper::Initialize() {
    simulator_->StartSim(interface_, arm_interface_, arm2_interface_);
    running_ = true;
}

//Draco Walking Methods

void DracoWrapper::SetWalkRawCommand(double ft_length, double r_ft_width, double l_ft_width,
                                  double ori_inc, int num_step) {
    if (!running_) {
        throw std::bad_function_call();
    }
    while(!interface_->IsReadyForNextCommand()) {
        std::this_thread::sleep_for (std::chrono::milliseconds (100));
    }
    interface_->Walk(ft_length, r_ft_width, l_ft_width, ori_inc, num_step);
}

void DracoWrapper::SetWalkXCommand(double x) {
    if (!running_) {
        throw std::bad_function_call();
    }
    while(!interface_->IsReadyForNextCommand()) {
        std::this_thread::sleep_for (std::chrono::milliseconds (100));
    }
    interface_->WalkInX(x);
}

void DracoWrapper::SetWalkYCommand(double y) {
    if (!running_) {
        throw std::bad_function_call();
    }
    while(!interface_->IsReadyForNextCommand()) {
        std::this_thread::sleep_for (std::chrono::milliseconds (100));
    }
    interface_->WalkInY(y);
}

void DracoWrapper::SetTurnCommand( double th) {
    if (!running_) {
        throw std::bad_function_call();
    }
    while(!interface_->IsReadyForNextCommand()) {
        std::this_thread::sleep_for (std::chrono::milliseconds (100));
    }
    interface_->Turn(th);
}

void DracoWrapper::SetWalkToRelativeCommand(double x, double y, double th) {
    if (!running_) {
        throw std::bad_function_call();
    }
    while(!interface_->IsReadyForNextCommand()) {
        std::this_thread::sleep_for (std::chrono::milliseconds (100));
    }
    interface_->WalkToRelativePositionAndOrientation(x, y, th);
}

void DracoWrapper::SetHaltCommand() {
    myUtils::color_print(myColor::BoldRed,
                         "[[Halting]]");
}

//Scorpio Manipulation Methods
//Scorpio Manipulation Methods

void DracoWrapper::SetMoveEndEffectorCommand(char *arm, double x, double y, double z){
    if (!running_) {
        throw std::bad_function_call();
    }
    if (ARM1_NAME.compare(arm) == 0) {
        while(!arm_interface_->IsReadyToMove()) {
            std::this_thread::sleep_for (std::chrono::milliseconds (100));
        }
        myUtils::color_print(myColor::BoldRed,
                             "[[MOVING(1) to" + std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) + "]]");
        arm_interface_->MoveEndEffectorTo(x, y, z);
    } else {
        while(!arm2_interface_->IsReadyToMove()) {
            std::this_thread::sleep_for (std::chrono::milliseconds (100));
        }
        myUtils::color_print(myColor::BoldRed,
                             "[[MOVING(2) to" + std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) + "]]");
        arm2_interface_->MoveEndEffectorTo(x, y, z);
    }
}

void DracoWrapper::SetCloseGripperCommand(char *arm){
    if (!running_) {
        throw std::bad_function_call();
    }
    if (ARM1_NAME.compare(arm) == 0) {
        while (!arm_interface_->IsReadyToGrasp()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        arm_interface_->Grasp();
    } else {
        while (!arm2_interface_->IsReadyToGrasp()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        arm2_interface_->Grasp();
    }
}

void DracoWrapper::SetOpenGripperCommand(char *arm){
    if (!running_) {
        throw std::bad_function_call();
    }
    if (ARM1_NAME.compare(arm) == 0) {
        while (!arm_interface_->IsReadyToRelease()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        arm_interface_->Release();
    } else {
        while (!arm2_interface_->IsReadyToRelease()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        arm2_interface_->Release();
    }
}

void DracoWrapper::Shutdown() {
    simulator_->StopSim();
}


