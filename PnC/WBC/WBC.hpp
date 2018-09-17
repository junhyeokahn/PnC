#ifndef WHOLE_BODY_CONTROLLER
#define WHOLE_BODY_CONTROLLER

#include <Utils/Utilities.hpp>
#include <Utils/pseudo_inverse.hpp>
#include "PnC/WBC/Task.hpp"
#include "PnC/WBC/WBLC/WBLCContact.hpp"

// Assume first 6 (or 3 in 2D case) joints are for the representation of 
// a floating base.
class WBC{
    public:
        WBC(const std::vector<bool> & act_list, 
                const Eigen::MatrixXd * Jc_internal = NULL):
            num_act_joint_(0),
            num_passive_(0),
            b_internal_constraint_(false)
    {
        num_qdot_ = act_list.size();
        for(int i(0); i<num_qdot_; ++i){
            if(act_list[i] == true) ++num_act_joint_;
            else ++num_passive_;
        }
        Sa_ = Eigen::MatrixXd::Zero(num_act_joint_, num_qdot_);
        Sv_ = Eigen::MatrixXd::Zero(num_passive_, num_qdot_);

        // Set virtual & actuated selection matrix
        int j(0);
        int k(0);
        for(int i(0); i <num_qdot_; ++i){
            if(act_list[i] == true){
                Sa_(j, i) = 1.;
                ++j;
            }
            else{
                Sv_(k,i) = 1.;
                ++k;
            }
        }

        if(Jc_internal){
            Jci_ = *Jc_internal;
            b_internal_constraint_ = true;
        }

    }
        virtual ~WBC(){}

        virtual void UpdateSetting(const Eigen::MatrixXd & A,
                const Eigen::MatrixXd & Ainv,
                const Eigen::VectorXd & cori,
                const Eigen::VectorXd & grav,
                void* extra_setting = NULL) = 0;

        virtual void MakeTorque(const std::vector<Task*> & task_list,
                const std::vector<WBLCContact*> & contact_list,
                Eigen::VectorXd & cmd,
                void* extra_input = NULL) =0;

    protected:
        // full rank fat matrix only
        void _WeightedInverse(const Eigen::MatrixXd & J,
                const Eigen::MatrixXd & Winv,
                Eigen::MatrixXd & Jinv){
            Eigen::MatrixXd lambda(J* Winv * J.transpose());
            Eigen::MatrixXd lambda_inv;
            myUtils::pseudoInverse(lambda, 0.0001, lambda_inv);
            Jinv = Winv * J.transpose() * lambda_inv;
        }

        int num_qdot_;
        int num_act_joint_;
        int num_passive_;

        Eigen::MatrixXd Sa_; // Actuated joint
        Eigen::MatrixXd Sv_; // Virtual joint

        Eigen::MatrixXd A_;
        Eigen::MatrixXd Ainv_;
        Eigen::VectorXd cori_;
        Eigen::VectorXd grav_;

        bool b_updatesetting_;

        bool b_internal_constraint_;
        Eigen::MatrixXd Jci_; // internal constraint Jacobian
        Eigen::MatrixXd Nci_;
};

#endif
