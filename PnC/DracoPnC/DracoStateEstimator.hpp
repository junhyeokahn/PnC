#pragma once

#include <Eigen/Dense>
#include <Configuration.h>

class DracoStateProvider;
class RobotSystem;
class DracoSensorData;

class DracoStateEstimator {
   public:
    DracoStateEstimator(RobotSystem* robot);
    ~DracoStateEstimator();

    void Initialization(DracoSensorData*);
    void Update(DracoSensorData*);

   protected:
    DracoStateProvider* sp_;
    RobotSystem* robot_;

    Eigen::VectorXd curr_config_;
    Eigen::VectorXd curr_qdot_;

    void _JointUpdate(DracoSensorData* data);
    void _ConfigurationAndModelUpdate();
    void _FootContactUpdate(DracoSensorData* data);
};
