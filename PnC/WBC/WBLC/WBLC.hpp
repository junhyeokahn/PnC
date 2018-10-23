#pragma once

#include <Utils/Utilities.hpp>
#include <ExternalSource/myOptimizer/Goldfarb/QuadProg++.hh>
#include <PnC/WBC/WBC.hpp>
#include <PnC/WBC/ContactSpec.hpp>

class WBLC_ExtraData{
    public:
        // Output
        Eigen::VectorXd opt_result_;
        Eigen::VectorXd qddot_;
        Eigen::VectorXd Fr_;

        // Input
        Eigen::VectorXd W_qddot_;
        Eigen::VectorXd W_rf_;
        Eigen::VectorXd W_xddot_;

        Eigen::VectorXd tau_min_;
        Eigen::VectorXd tau_max_;

        WBLC_ExtraData(){}
        ~WBLC_ExtraData(){}
};

class WBLC: public WBC{
    public:
        WBLC(const std::vector<bool> & act_list, const Eigen::MatrixXd* Jci = NULL);
        virtual ~WBLC(){}

        virtual void updateSetting(const Eigen::MatrixXd & A,
                const Eigen::MatrixXd & Ainv,
                const Eigen::VectorXd & cori,
                const Eigen::VectorXd & grav,
                void* extra_setting = NULL);

        void makeWBLC_Torque(const Eigen::VectorXd & des_jacc_cmd,
                const std::vector<ContactSpec*> & contact_list,
                Eigen::VectorXd & cmd,
                void* extra_input = NULL);

        virtual void makeTorque(
                const std::vector<Task*> & task_list,
                const std::vector<ContactSpec*> & contact_list,
                Eigen::VectorXd & cmd,
                void* extra_input = NULL);



    private:
        std::vector<int> act_list_;

        void _OptimizationPreparation(
                const Eigen::MatrixXd & Aeq,
                const Eigen::VectorXd & beq,
                const Eigen::MatrixXd & Cieq,
                const Eigen::VectorXd & dieq);


        void _GetSolution(Eigen::VectorXd & cmd);
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

        // BuildContactMtxVect builds the followings:
        void _BuildContactMtxVect(const std::vector<ContactSpec*> & contact_list);
        Eigen::MatrixXd Uf_;
        Eigen::VectorXd Fr_ieq_;
        Eigen::MatrixXd Jc_;
        Eigen::VectorXd JcDotQdot_;

        // Setup the followings:
        void _Build_Equality_Constraint();
        Eigen::MatrixXd Aeq_;
        Eigen::VectorXd beq_;

        void _Build_Inequality_Constraint();
        Eigen::MatrixXd Cieq_;
        Eigen::VectorXd dieq_;

        Eigen::VectorXd qddot_;

        Eigen::MatrixXd Sf_; //floating base
        void _PrintDebug(double i) {
            //printf("[WBLC] %f \n", i);
        }
};
