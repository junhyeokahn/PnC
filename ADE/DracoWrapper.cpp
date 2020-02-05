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

void DracoWrapper::SetWalkCommand(double ft_length, double r_ft_width, double l_ft_width,
                                  double ori_inc, int num_step) {
    if (!running_) {
        throw std::bad_function_call();
    }
    //Todo some sort of conversion from the heading to the walking params
    //Dummy single step numbers for now
    interface_->Walk(ft_length, r_ft_width, l_ft_width, ori_inc, num_step);
}

void DracoWrapper::Shutdown() {
    simulator_->StopSim();
}


