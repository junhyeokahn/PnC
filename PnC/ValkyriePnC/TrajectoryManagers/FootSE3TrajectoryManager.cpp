#include <PnC/ValkyriePnC/TrajectoryManagers/FootSE3TrajectoryManager.hpp>

FootSE3TrajectoryManager::FootSE3TrajectoryManager(Task* _foot_pos_task, Task* _foot_ori_task, RobotSystem* _robot): TrajectoryManagerBase(_robot){
	// Set Linear and Orientation Foot task
	foot_pos_task_ = _foot_pos_task;
	foot_ori_task_ = _foot_ori_task;
}

FootSE3TrajectoryManager::~FootSE3TrajectoryManager(){
}

void FootSE3TrajectoryManager::paramInitialization(const YAML::Node& node){	
}