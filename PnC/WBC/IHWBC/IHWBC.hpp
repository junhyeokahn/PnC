#include <PnC/WBC/Task.hpp>
#include <PnC/WBC/ContactSpec.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <ExternalSource/myOptimizer/Goldfarb/QuadProg++.hh>

// Implicit Hierarchical Whole Body Controller
class IHWBC{
public:
	IHWBC(const std::vector<bool> & act_list);
    virtual ~IHWBC();

    void updateSetting(const Eigen::MatrixXd & A,
                    const Eigen::MatrixXd & Ainv,
                    const Eigen::VectorXd & cori,
                    const Eigen::VectorXd & grav,
                    void* extra_setting = NULL);

    void solve(const std::vector<Task*> & task_list,
               const std::vector<ContactSpec*> & contact_list,
               const Eigen::VectorXd & Fd,                
               Eigen::VectorXd & tau_cmd, Eigen::VectorXd & qddot_cmd);

    // Weight of task hierarchies
    Eigen::VectorXd w_task_heirarchy;

    // Weights of contact reaction forces
    Eigen::VectorXd w_rf_contacts;

    // Tikhonov regularization
    double lambda_qddot; 
    double lambda_Fr; 

private:
    int num_qdot_; // Number of degrees of freedom
    int num_act_joint_; // Actuated Joints
    int num_passive_; // Passive Joints

    // Selection Matrices
    Eigen::MatrixXd Sa_; // Actuated joint
    Eigen::MatrixXd Sv_; // Virtual joint
    Eigen::MatrixXd Sf_; // Floating base 

    Eigen::MatrixXd A_;
    Eigen::MatrixXd Ainv_;
    Eigen::VectorXd cori_;
    Eigen::VectorXd grav_;	

    bool b_updatesetting_;

    // Quadprog Variables
    GolDIdnani::GVect<double> x;
    // Cost
    GolDIdnani::GMatr<double> G;
    GolDIdnani::GVect<double> g0;

    // Equality
    GolDIdnani::GMatr<double> CE;
    GolDIdnani::GVect<double> ce0;

    // Inequality
    GolDIdnani::GMatr<double> CI;
    GolDIdnani::GVect<double> ci0;

};

