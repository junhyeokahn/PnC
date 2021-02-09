#pragma once

#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/pseudo_inverse.hpp>
#include <ExternalSource/myOptimizer/Goldfarb/QuadProg++.hh>
#include <PnC/WBC/ContactSpec.hpp>
#include <PnC/WBC/Task.hpp>

class WBIC_ExtraData{
    public:
        // Output
        Eigen::VectorXd _opt_result;
        Eigen::VectorXd _qddot;
        Eigen::VectorXd _Fr;

        // Input
        Eigen::VectorXd _W_floating;
        Eigen::VectorXd _W_rf;
        //Eigen::VectorXd W_xddot_;

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

        Eigen::MatrixXd Sa_; // Actuated Joint Selection
        Eigen::MatrixXd Sv_; // Virtual Joint Selection

        Eigen::MatrixXd A_;
        Eigen::MatrixXd Ainv_;
        Eigen::VectorXd cori_;
        Eigen::VectorXd grav_;

        bool b_updatesetting_;
        bool b_internal_constraint_;

    private:
        std::vector<Task*> _task_list;
        std::vector<ContactSpec*> _contact_list;

        void _SetEqualityConstraint(const Eigen::VectorXd & qddot);
        void _SetInequalityConstraint();
        void _ContactBuilding();

        void _GetSolution(const Eigen::VectorXd & qddot, Eigen::VectorXd & cmd);
        void _SetCost();
        void _SetOptimizationSize();

        int num_qdot_;
        int num_act_joint_;
        int num_passive_;
        int _dim_floating;

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
        Eigen::MatrixXd _dyn_CI;
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
