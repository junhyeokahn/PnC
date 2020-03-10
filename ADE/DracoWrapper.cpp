#include <PnC/DracoPnC/DracoInterface.hpp>
#include <Simulator/Dart/Scorpio/ScorpioWorldNode.hpp>
#include <ADE/DracoWrapper.hpp>
#include <thread>
#include <chrono>



DracoWrapper::DracoWrapper() : running_(false) {
    interface_ = new DracoInterface();
    arm_interface_ = new ScorpioInterface();
    arm2_interface_ = new Scorpio2Interface();
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
    simulator_->viewer.simulate(true);
    interface_->Walk(ft_length, r_ft_width, l_ft_width, ori_inc, num_step);
    while(!interface_->IsReadyForNextCommand()) {
        std::this_thread::sleep_for (std::chrono::milliseconds (SLEEP_MILLIS));
    }
    simulator_->viewer.simulate(false);
}

void DracoWrapper::SetWalkXCommand(double x) {
    if (!running_) {
        throw std::bad_function_call();
    }
    simulator_->viewer.simulate(true);
    interface_->WalkInX(x);
    while(!interface_->IsReadyForNextCommand()) {
        std::this_thread::sleep_for (std::chrono::milliseconds (SLEEP_MILLIS));
    }
    simulator_->viewer.simulate(false);
}

void DracoWrapper::SetWalkYCommand(double y) {
    if (!running_) {
        throw std::bad_function_call();
    }
    simulator_->viewer.simulate(true);
    interface_->WalkInY(y);
    while(!interface_->IsReadyForNextCommand()) {
        std::this_thread::sleep_for (std::chrono::milliseconds (SLEEP_MILLIS));
    }
    simulator_->viewer.simulate(false);
}

void DracoWrapper::SetTurnCommand( double th) {
    if (!running_) {
        throw std::bad_function_call();
    }
    simulator_->viewer.simulate(true);
    interface_->Turn(th);
    while(!interface_->IsReadyForNextCommand()) {
        std::this_thread::sleep_for (std::chrono::milliseconds (SLEEP_MILLIS));
    }
    simulator_->viewer.simulate(false);
}

void DracoWrapper::SetWalkToRelativeCommand(double x, double y, double th) {
    if (!running_) {
        throw std::bad_function_call();
    }
    simulator_->viewer.simulate(true);
    interface_->WalkToRelativePositionAndOrientation(x, y, th);
    while(!interface_->IsReadyForNextCommand()) {
        std::this_thread::sleep_for (std::chrono::milliseconds (SLEEP_MILLIS));
    }
    simulator_->viewer.simulate(false);
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
        myUtils::color_print(myColor::BoldRed,
                             "[[MOVING(1) to" + std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) + "]]");
        simulator_->viewer.simulate(true);
        arm_interface_->MoveEndEffectorTo(x, y, z);
        while(!arm_interface_->IsReadyToMove()) {
            std::this_thread::sleep_for (std::chrono::milliseconds (SLEEP_MILLIS));
        }
        simulator_->viewer.simulate(false);
    } else {
        myUtils::color_print(myColor::BoldRed,
                             "[[MOVING(2) to" + std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) + "]]");
        simulator_->viewer.simulate(true);
        arm2_interface_->MoveEndEffectorTo(x, y, z);
        while(!arm2_interface_->IsReadyToMove()) {
            std::this_thread::sleep_for (std::chrono::milliseconds (SLEEP_MILLIS));
        }
        simulator_->viewer.simulate(false);
    }
}

void DracoWrapper::SetCloseGripperCommand(char *arm){
    myUtils::color_print(myColor::BoldRed,
                         "[[About to grasp]]");
    if (!running_) {
        throw std::bad_function_call();
    }
    if (ARM1_NAME.compare(arm) == 0) {
        myUtils::color_print(myColor::BoldRed,
                             "[[GRASPING]]");
//        while (!arm_interface_->IsReadyToGrasp() || !arm_interface_->IsReadyToMove()) {
//            std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_MILLIS));
//        }
        simulator_->viewer.simulate(true);
        arm_interface_->Grasp();
        while (!arm_interface_->IsReadyToMove()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_MILLIS));
        }
        simulator_->node->box_ph = BoxPH::scorpio;
        simulator_->viewer.simulate(false);
    } else {
        myUtils::color_print(myColor::BoldRed,
                             "[[GRASPING(2)]]");
//        while (!arm2_interface_->IsReadyToGrasp() || !arm_interface_->IsReadyToMove()) {
//            std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_MILLIS));
//        }
        simulator_->viewer.simulate(true);
        arm2_interface_->Grasp();
        while (!arm2_interface_->IsReadyToMove()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_MILLIS));
        }
        simulator_->node->box_ph = BoxPH::scorpio2;
        simulator_->viewer.simulate(false);
    }
}

void DracoWrapper::SetOpenGripperCommand(char *arm){
    myUtils::color_print(myColor::BoldRed,
                         "[[About to release]]");
    if (!running_) {
        throw std::bad_function_call();
    }
    if (ARM1_NAME.compare(arm) == 0) {
        myUtils::color_print(myColor::BoldRed,
                             "[[RELEASING]]");
        simulator_->viewer.simulate(true);
        while (!arm_interface_->IsReadyToMove()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_MILLIS));
        }
        //arm_interface_->Release();
        simulator_->node->box_ph = BoxPH::draco;
        simulator_->viewer.simulate(false);
    } else {
        myUtils::color_print(myColor::BoldRed,
                             "[[RELEASING(2)]]");
        simulator_->viewer.simulate(true);
        while (!arm2_interface_->IsReadyToMove()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_MILLIS));
        }
        //arm2_interface_->Release();
        simulator_->node->box_ph = BoxPH::table2;
        simulator_->viewer.simulate(false);
    }
}

void DracoWrapper::Shutdown() {
    simulator_->StopSim();
}


