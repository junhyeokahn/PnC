#pragma once

#include <Configuration.h>
#include <Eigen/Dense>

class ValkyrieStateProvider;
class RobotSystem;
class ValkyrieSensorData;

class ValkyrieStateEstimator {
   public:
    ValkyrieStateEstimator(RobotSystem* robot);
    ~ValkyrieStateEstimator();

    void Initialization(ValkyrieSensorData*);
    void Update(ValkyrieSensorData*);

   protected:
    ValkyrieStateProvider* sp_;
    RobotSystem* robot_;

    Eigen::VectorXd curr_config_;
    Eigen::VectorXd curr_qdot_;

    void _JointUpdate(ValkyrieSensorData* data);
    void _ConfigurationAndModelUpdate();
    void _FootContactUpdate(ValkyrieSensorData* data);
};
