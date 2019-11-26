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


    // Returns the joint acceleration computed by the QP
    void getQddotResult(Eigen::VectorXd & qddot_out);
    // Returns the reaction forces computed by the QP
    void getFrResult(Eigen::VectorXd & Fr_out);

    // w_task_heirarchy_in:
    //  - sets relative weight between task priority
    //  - must have dimension equal to the number of tasks.
    // w_rf_contacts_in (for target wrench minimization only)
    //  - note that ||w_rf_contacts_in||_1 = 1.0
    //      - ie: sum of w_rf_contacts[i] = 1.0
    //      - a higher number on the i-th contact will put more force on that contact
    //  - sets relative weight to distribute the foces between contacts 
    //  - must have dimension equal to the number of contacts
    //
    // w_contact_weight_in  
    //  - sets the relative weight of the contact forces and the task hierarchies
    void setQPWeights(const Eigen::VectorXd & w_task_heirarchy_in, const Eigen::VectorXd & w_rf_contacts_in, const double & w_contact_weight_in);
    void setQPWeights(const Eigen::VectorXd & w_task_heirarchy_in, const double & w_contact_weight_in);
    void setRegularizationTerms(const double lambda_qddot_in, const double lambda_Fr_in); 

    void setTorqueLimits(const Eigen::VectorXd & tau_min_in, const Eigen::VectorXd & tau_max_in);
    void enableTorqueLimits(const bool b_torque_limit_in);

    // If true, we try to minimize for a target wrench value. Fd \in mathbf{R}^6.
    // If false, we try to minimize the desired contact forces term by term: Fd \in mathbf{R}^(n). n = dim of reaction force
    void setTargetWrenchMinimization(const bool target_wrench_minimization_in); 
    bool target_wrench_minimization;

    // Weight of task hierarchies
    Eigen::VectorXd w_task_heirarchy;
    // Weights of contact reaction forces
    Eigen::VectorXd w_rf_contacts;
    double w_contact_weight;

    // Tikhonov regularization
    double lambda_qddot; 
    double lambda_Fr; 

private:
    int num_qdot_; // Number of degrees of freedom
    int num_act_joint_; // Actuated Joints
    int num_passive_; // Passive Joints
    int dim_contacts_; // Dimension of Contacts
    int dim_contact_constraints_; // Dimension of Contact Constraints

    int dim_dec_vars_;
    int dim_inequality_constraints_;

    // Selection Matrices
    Eigen::MatrixXd Sa_; // Actuated joint
    Eigen::MatrixXd Sv_; // Virtual joint
    Eigen::MatrixXd Sf_; // Floating base 

    Eigen::MatrixXd A_;
    Eigen::MatrixXd Ainv_;
    Eigen::VectorXd cori_;
    Eigen::VectorXd grav_;	

    // Contact Jacobians
    Eigen::MatrixXd Jc_;
    std::vector<Eigen::MatrixXd> Jc_list_;

    //Contact Constraints
    Eigen::MatrixXd Uf_;
    Eigen::VectorXd uf_ieq_vec_;

    // Set Torque limits
    Eigen::VectorXd tau_min_;
    Eigen::VectorXd tau_max_;    

    bool b_weights_set_;
    bool b_updatesetting_;
    bool b_torque_limits_;

    // Quadprog sizes
    int n_quadprog_ = 1; // Number of Decision Variables
    int p_quadprog_ = 0; // Number of Inequality Constraints
    int m_quadprog_ = 0; // Number of Equality Constraints

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


    // Quadprog Result Containers
    Eigen::VectorXd qp_dec_vars_;
    Eigen::VectorXd qddot_result_;
    Eigen::VectorXd Fr_result_;

    void buildContactStacks(const std::vector<ContactSpec*> & contact_list, const Eigen::VectorXd & w_rf_contacts_in);

    void prepareQPSizes();
    void setQuadProgCosts(const Eigen::MatrixXd & P_cost, const Eigen::VectorXd & v_cost);
    void setEqualityConstraints(const Eigen::MatrixXd & Eq_mat, const Eigen::VectorXd & Eq_vec);
    void setInequalityConstraints(const Eigen::MatrixXd & IEq_mat, const Eigen::VectorXd & IEq_vec);
    void solveQP();

};

