#include <PnC/DracoPnC/DracoInterface.hpp>
#include <ADE/DracoWrapper.hpp>
#include <thread>



DracoWrapper::DracoWrapper() : running_(false) {
    interface_ = new DracoInterface();
    simulator_ = new DracoSim(interface_);
}

void DracoWrapper::Initialize() {
    simulator_->StartSim();
    running_ = true;
}

void DracoWrapper::SetWalkCommand() {
    if (!running_) {
        throw std::bad_function_call();
    }
    //Todo some sort of conversion from the heading to the walking params
    //Dummy single step numbers for now
    interface_->Walk(0., 0.33, 0.3, 0., 7);
}

void DracoWrapper::Shutdown() {
    simulator_->StopSim();
}


