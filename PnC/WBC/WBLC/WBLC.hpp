#ifndef WHOLE_BODY_LOCOMOTION_CONTROL_H
#define WHOLE_BODY_LOCOMOTION_CONTROL_H

#include <WBC.hpp>
#include <Utils/Utilities.hpp>
#include <Optimizer/Goldfarb/QuadProg++.hh>

#include <Task.hpp>
#include "WBLCContact.hpp"

class WBLC_ExtraData{
    public:
        Eigen::VectorXd cost_weight;
        Eigen::VectorXd opt_result_;

        WBLC_ExtraData(){}
        ~WBLC_ExtraData(){}
};

class WBLC: public WBC{
    public:
        WBLC(const std::vector<bool> & act_list, const Eigen::MatrixXd* Jci = NULL);
        virtual ~WBLC(){}

        virtual void UpdateSetting(const Eigen::MatrixXd & A,
                const Eigen::MatrixXd & Ainv,
                const Eigen::VectorXd & cori,
                const Eigen::VectorXd & grav,
                void* extra_setting = NULL);

        virtual void MakeTorque(const std::vector<Task*> & task_list,
                const std::vector<WBLCContact*> & contact_list,
                Eigen::VectorXd & cmd,
                void* extra_input = NULL);

    private:
        void _SetInEqualityConstraint();
        void _ContactBuilding(const std::vector<WBLCContact*> & contact_list);

        void _GetSolution(const Eigen::VectorXd & qddot, Eigen::VectorXd & cmd);
        bool _CheckNullSpace(const Eigen::MatrixXd & Npre);
        void _OptimizationPreparation();

        int dim_opt_;
        int dim_eq_cstr_; // equality constraints
        int dim_ieq_cstr_; // inequality constraints
        int dim_first_task_; // first task dimension
        WBLC_ExtraData* data_;

        GolDIdnani::GVect<double> z;
        // Cost
        GolDIdnani::GMatr<double> G;
        GolDIdnani::GVect<double> g0;

        // Equality
        GolDIdnani::GMatr<double> CE;
        GolDIdnani::GVect<double> ce0;

        // Inequality
        GolDIdnani::GMatr<double> CI;
        GolDIdnani::GVect<double> ci0;

        int dim_rf_;
        int dim_relaxed_task_;
        int dim_cam_;
        int dim_rf_cstr_;

        Eigen::MatrixXd tot_tau_Mtx_;
        Eigen::VectorXd tot_tau_Vect_;

        Eigen::MatrixXd S_delta_;
        Eigen::MatrixXd Uf_;
        Eigen::VectorXd uf_ieq_vec_;

        Eigen::MatrixXd Jc_;
        Eigen::VectorXd JcDotQdot_;

        Eigen::MatrixXd B_;
        Eigen::VectorXd c_;
        Eigen::VectorXd task_cmd_;

        Eigen::MatrixXd Sf_; //floating base
        void _PrintDebug(double i) {
            //printf("[WBLC] %f \n", i);
        }
};

#endif
