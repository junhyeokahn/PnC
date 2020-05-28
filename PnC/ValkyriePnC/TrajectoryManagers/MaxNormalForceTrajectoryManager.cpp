#include <PnC/ValkyriePnC/TrajectoryManagers/MaxNormalForceTrajectoryManager.hpp>

MaxNormalForceTrajectoryManager::MaxNormalForceTrajectoryManager(ContactSpec* _contact, RobotSystem* _robot): TrajectoryManagerBase(_robot){
    myUtils::pretty_constructor(2, "TrajectoryManager: Max Normal Force");
    contact_ = _contact;
    nominal_max_normal_force_z_ = 1500; // Default
    starting_max_normal_force_z_ = 0.0; // Default
    current_max_normal_force_z_ = 0.0; // Default
    local_max_normal_force_z_ = 0.0;
}

MaxNormalForceTrajectoryManager::~MaxNormalForceTrajectoryManager(){    
}

void MaxNormalForceTrajectoryManager::paramInitialization(const YAML::Node& node){
    try {
        // Load Maximum normal force
        myUtils::readParameter(node, "max_z_force", nominal_max_normal_force_z_);

    } catch(std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
        exit(0);
    }
}

void MaxNormalForceTrajectoryManager::initializeRampToZero(const double start_time, const double nominal_ramp_duration){
    // Initialize start times and starting max value
    ramp_start_time_ = start_time;
    starting_max_normal_force_z_ = current_max_normal_force_z_;
    // Initialize ramp speed
    ramp_down_speed_ = -nominal_max_normal_force_z_ / nominal_ramp_duration;
}

void MaxNormalForceTrajectoryManager::initializeRampToMax(const double start_time, const double nominal_ramp_duration){
    // Initialize start times and starting max value
    ramp_start_time_ = start_time;
    starting_max_normal_force_z_ = current_max_normal_force_z_;
    // Initialize ramp speed
    ramp_up_speed_ = nominal_max_normal_force_z_ / nominal_ramp_duration;       
}

void MaxNormalForceTrajectoryManager::computeRampToZero(const double current_time){
    local_max_normal_force_z_ = ramp_down_speed_*(current_time - ramp_start_time_) + starting_max_normal_force_z_;
    current_max_normal_force_z_ = myUtils::CropValue(local_max_normal_force_z_, 0.0, nominal_max_normal_force_z_);
}

void MaxNormalForceTrajectoryManager::computeRampToMax(const double current_time){
    local_max_normal_force_z_ = ramp_up_speed_*(current_time - ramp_start_time_) + starting_max_normal_force_z_;
    current_max_normal_force_z_ = myUtils::CropValue(local_max_normal_force_z_, 0.0, nominal_max_normal_force_z_);    
}

void MaxNormalForceTrajectoryManager::updateMaxNormalForce(){
    ((SurfaceContactSpec*)contact_)->setMaxFz(current_max_normal_force_z_);    
}
