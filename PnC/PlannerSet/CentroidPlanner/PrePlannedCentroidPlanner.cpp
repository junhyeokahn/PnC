#include "PrePlannedCentroidPlanner.hpp"

PrePlannedCentroidPlanner::PrePlannedCentroidPlanner() {

}

PrePlannedCentroidPlanner::~PrePlannedCentroidPlanner() {}

void PrePlannedCentroidPlanner::_doPlan() {
    try {
        Eigen::MatrixXd dummyMat;
        YAML::Node trajectory_cfg =
            YAML::LoadFile(mPreComputedParam->trajectoryFile);
    } catch(std::runtime_error& e) {
        std::cout << "Error reading parameter ["<< e.what() << "] at file: [" << __FILE__ << "]" << std::endl << std::endl;
    }
}

void PrePlannedCentroidPlanner::_evalTrajectory( double time,
                                                 Eigen::VectorXd & pos,
                                                 Eigen::VectorXd & vel,
                                                 Eigen::VectorXd & trq) {

}
