#include <PnC/WBC/ContactSpec.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <ExternalSource/myOptimizer/Goldfarb/QuadProg++.hh>

// Implicit Hierarchical Whole Body Controller
class IHWBC{
public:
	IHWBC(const std::vector<bool> & act_list);
    virtual ~IHWBC();

protected:
    int num_qdot_; // Number of degrees of freedom
    int num_act_joint_; // Actuated Joints
    int num_passive_; // Passive Joints

    Eigen::MatrixXd Sa_; // Actuated joint
    Eigen::MatrixXd Sv_; // Virtual joint

    Eigen::MatrixXd A_;
    Eigen::MatrixXd Ainv_;
    Eigen::VectorXd cori_;
    Eigen::VectorXd grav_;	

};

