#pragma once

#include <Utils/IO/IOUtilities.hpp>
#include <ExternalSource/myOptimizer/Goldfarb/QuadProg++.hh>
#include <PnC/WBC/ContactSpec.hpp>

class WBIC_ExtraData{
    public:
        // Output
        Eigen::VectorXd opt_result_;
        Eigen::VectorXd qddot_;
        Eigen::VectorXd Fr_;

        // Input
        Eigen::VectorXd W_qddot_;
        Eigen::VectorXd W_rf_;
        Eigen::VectorXd W_xddot_;

        WBIC_ExtraData(){}
        ~WBIC_ExtraData(){}
};

class WBIC{
    public:
        WBIC(const std::vector<bool> & act_list, const std::vector<Task*> task_list_, const std::vector<ContactSpec*> contact_list_);
        virtual ~WBIC(){}

        virtual void updateSetting(const Eigen::MatrixXd & A,
                                   const Eigen::MatrixXd & Ainv,
                                   const Eigen::VectorXd & cori,
                                   const Eigen::VectorXd & grav,
                                   void* extra_setting = NULL);

        virtual void makeTorque(
                const std::vector<Task*> & task_list,
                const std::vector<ContactSpec*> & contact_list,
                Eigen::VectorXd & cmd,
                void* extra_input = NULL);

        Eigen::MatrixXd Sa_ // Actuated Joint Selection
        Eigen::MatrixXd Sv_; // Virtual Joint Selection

        Eigen::MatrixXd A_;
        Eigen::MatrixXd Ainv_;
        Eigen::VectorXd cori_;
        Eigen::VectorXd grav_;

        bool b_updatesetting_;
        bool b_internal_constraint_;

    protected:
        void _WeightedInverse(const Eigen::MatrixXd & J, const Eigen::MatrixXd * Winv, Eigen::MatrixXd & Jinv,
                              double threshold = 0.0001) {
            Eigen::MatrixXd lambda = J*Winv*J.transpose();
            Eigen::MatrixXd lambda_inv;
            pseudoInverse(lambda, threshold, lambda_inv);
            Jinv = Winv * J.transpose() * lambda_inv;
        }

    private:
        std::vector<Task*> _task_list;
        std::vector<ContactSpec*> _contact_list;

        void _SetEqualityConstraint(const Eigen::VectorXd & qddot);
        void _SetInequalityConstraint();
        void _ContactBuilding();

        void _GetSolution(const Eigen::VectorXd & qddot, Eigen::VectorXd & cmd)
        void _SetCost();
        void _SetOptimizationSize();

        int _dim_opt; // Contact pt delta, first task delta, rxn force
        int _dim_eq_cstr; // equality constraints
        int _dim_rf; // inequality constraints
        int _dim_Uf;

        WBIC_ExtraData* _data;

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

        Eigen::MatrixXd _dyn_CE;
        Eigen::VectorXd _dyn_ce0;
        Eigen::MatrxXd _dyn_CI;
        Eigen::VectorXd _dyn_ci0;

        Eigen::MatrixXd _eye;
        Eigen::MatrixXd _eye_floating;

        Eigen::MatrixXd _S_delta;
        Eigen::MatrixXd _Uf;
        Eigen::VectorXd _Uf_ieq_vec;

        Eigen::MatrixXd _Jc;
        Eigen::VectorXd _JcDotQdot;
        Eigen::VectorXd _Fr_des;

        Eigen::MatrixXd _B;
        Eigen::VectorXd _C;
        Eigen::VectorXd task_cmd;
};
