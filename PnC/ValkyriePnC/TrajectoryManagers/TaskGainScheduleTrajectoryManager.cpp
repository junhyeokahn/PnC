#include <PnC/ValkyriePnC/TrajectoryManagers/TaskGainScheduleTrajectoryManager.hpp>

TaskGainScheduleTrajectoryManager::TaskGainScheduleTrajectoryManager(Task* _task, RobotSystem* _robot): TrajectoryManagerBase(_robot){
    myUtils::pretty_constructor(2, "TrajectoryManager: Task Gain Schedule");
    task_ = _task;
    nominal_w_max_ = 40; // Default
    nominal_w_min_ = 20; //Default
    starting_w_ = nominal_w_max_; // Default
    current_w_ = nominal_w_max_; // Default
    local_w_ = 0.0;
    nominal_ramp_duration_ = 1.0; //seconds
}

TaskGainScheduleTrajectoryManager::~TaskGainScheduleTrajectoryManager(){    
}

void TaskGainScheduleTrajectoryManager::paramInitialization(const YAML::Node& node){
    try {
        // Load Maximum normal force
        myUtils::readParameter(node, "w_task_foot_contact", nominal_w_max_);
        myUtils::readParameter(node, "w_task_foot_swing", nominal_w_min_);

    } catch(std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
        exit(0);
    }
}

void TaskGainScheduleTrajectoryManager::initializeRampToMin(const double start_time, const double nominal_ramp_duration){
    // Initialize start times and starting max value
    ramp_start_time_ = start_time;
    starting_w_ = current_w_;
    nominal_ramp_duration_ = nominal_ramp_duration;
    // Initialize ramp speed
    ramp_down_speed_ = -nominal_w_max_ / nominal_ramp_duration_;
}

void TaskGainScheduleTrajectoryManager::initializeRampToMax(const double start_time, const double nominal_ramp_duration){
    // Initialize start times and starting max value
    ramp_start_time_ = start_time;
    starting_w_ = current_w_;
    nominal_ramp_duration_ = nominal_ramp_duration;
    // Initialize ramp speed
    ramp_up_speed_ = nominal_w_max_ / nominal_ramp_duration_;       
}

void TaskGainScheduleTrajectoryManager::computeRampToMin(const double current_time){
    double t_current = myUtils::CropValue(current_time, ramp_start_time_, nominal_ramp_duration_);
    local_w_ = ramp_down_speed_*(t_current - ramp_start_time_) + starting_w_;
    current_w_ = myUtils::CropValue(local_w_, nominal_w_min_, nominal_w_max_);
}

void TaskGainScheduleTrajectoryManager::computeRampToMax(const double current_time){
    double t_current = myUtils::CropValue(current_time, ramp_start_time_, nominal_ramp_duration_);
    local_w_ = ramp_up_speed_*(t_current - ramp_start_time_) + starting_w_;
    current_w_ = myUtils::CropValue(local_w_, nominal_w_min_, nominal_w_max_);    
}


void TaskGainScheduleTrajectoryManager::updateRampToMinDesired(const double current_time){
    computeRampToMin(current_time);
    updateTaskHierarchy();
}
void TaskGainScheduleTrajectoryManager::updateRampToMaxDesired(const double current_time){
    computeRampToMax(current_time);
    updateTaskHierarchy();
}

void TaskGainScheduleTrajectoryManager::updateTaskHierarchy(){
    task_->setHierarchy(current_w_);    
}
