#include "WBLC.hpp"
#include <Utils/Utilities.hpp>
#include <Eigen/LU>
#include <Eigen/SVD>

WBLC::WBLC(const std::vector<bool> & act_list,
        const Eigen::MatrixXd * Jci): WBC(act_list, Jci){
    Sf_ = Eigen::MatrixXd::Zero(6, num_qdot_);
    Sf_.block(0,0, 6, 6).setIdentity();

    printf("[WBLC] Constructed\n");
}

void WBLC::UpdateSetting(const Eigen::MatrixXd & A,
        const Eigen::MatrixXd & Ainv,
        const Eigen::VectorXd & cori,
        const Eigen::VectorXd & grav,
        void* extra_setting){
    A_ = A;
    Ainv_ = Ainv;
    cori_ = cori;
    grav_ = grav;
    b_updatesetting_ = true;
}

bool WBLC::_CheckNullSpace(const Eigen::MatrixXd & Npre){
    Eigen::MatrixXd M_check = Sf_ * A_ * Npre;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M_check, Eigen::ComputeThinU | Eigen::ComputeThinV);
    //myUtils::pretty_print(svd.singularValues(), std::cout, "svd singular value");

    for(int i(0); i<svd.singularValues().rows(); ++i){
        if(svd.singularValues()[i] > 0.00001) {
            printf("non singular!!\n");
    //myUtils::pretty_print(svd.singularValues(), std::cout, "svd singular value");
            return false;
        }else{
            //printf("small enough singular value: %f\n", svd.singularValues()[i]);
        }
    }
    return true;
}

void WBLC::MakeTorque(const std::vector<Task*> & task_list,
        const std::vector<WBLCContact*> & contact_list,
        Eigen::VectorXd & cmd,
        void* extra_input){
    _PrintDebug(1);
    if(!b_updatesetting_) { printf("[Wanning] WBLC setting is not done\n"); }

    if(extra_input) data_ = static_cast<WBLC_ExtraData*>(extra_input);

    // Internal Constraint Check
    Nci_ = Eigen::MatrixXd::Identity(num_qdot_, num_qdot_);

    if(b_internal_constraint_) {
        Eigen::MatrixXd JciBar;
        _WeightedInverse(Jci_, Ainv_, JciBar);
        Nci_ -= JciBar * Jci_;
    }

    _PrintDebug(2);
    // Contact Setting
    _ContactBuilding(contact_list);
    Eigen::MatrixXd JcN = Jc_ * Nci_;
    Eigen::MatrixXd check = JcN - Jc_;

    _PrintDebug(3);
    Eigen::MatrixXd JcN_Bar;
    _WeightedInverse(JcN, Ainv_, JcN_Bar);
    Eigen::MatrixXd Npre = Eigen::MatrixXd::Identity(num_qdot_, num_qdot_)
        - JcN_Bar * JcN;

    _PrintDebug(4);
    // First Task Check
    Task* task = task_list[0];
    Eigen::MatrixXd Jt, JtPre, JtPreBar;
    Eigen::VectorXd JtDotQdot, xddot, qddot_pre;
    qddot_pre = JcN_Bar * ( - JcDotQdot_ );

    _PrintDebug(5);
    if(!task->IsTaskSet()){ printf("1st task is not set!\n"); exit(0); }
    task->getTaskJacobian(Jt);
    task->getTaskJacobianDotQdot(JtDotQdot);
    task->getCommand(xddot);
    //myUtils::pretty_print(xddot, std::cout, "command");
    dim_first_task_ = task->getDims();

    JtPre = Jt * Npre;
    _WeightedInverse(JtPre, Ainv_, JtPreBar);
    Npre = Npre * (Eigen::MatrixXd::Identity(num_qdot_, num_qdot_)
            - JtPreBar * JtPre);

    _PrintDebug(6);
    //_CheckNullSpace(Npre);
    // Optimization
    _PrintDebug(7);
    _OptimizationPreparation();
    _PrintDebug(8);
    // Set equality constraints
    Eigen::MatrixXd dyn_CE(dim_eq_cstr_, dim_opt_);
    Eigen::VectorXd dyn_ce0(dim_eq_cstr_);
    dyn_CE.block(0,0, dim_eq_cstr_, dim_first_task_) = Sf_ * A_ * JtPreBar;
    dyn_CE.block(0, dim_first_task_, dim_eq_cstr_, dim_rf_) = -Sf_ * JcN.transpose();
    dyn_ce0 = Sf_ * (A_ * (JtPreBar * (xddot - JtDotQdot - Jt * qddot_pre) + qddot_pre)
            + cori_ + grav_);
    for(int i(0); i< dim_eq_cstr_; ++i){
        for(int j(0); j<dim_opt_; ++j){
            CE[j][i] = dyn_CE(i,j);
        }
        ce0[i] = dyn_ce0[i];
    }
    _PrintDebug(9);
    // Set inequality constraints
    _SetInEqualityConstraint();
    _PrintDebug(10);

    // Optimization
    double f = solve_quadprog(G, g0, CE, ce0, CI, ci0, z);
    _PrintDebug(11);
    Eigen::VectorXd delta(dim_first_task_);
    for(int i(0); i<dim_first_task_; ++i) { delta[i] = z[i]; }
    qddot_pre = qddot_pre + JtPreBar * (xddot + delta - JtDotQdot - Jt * qddot_pre);
    _PrintDebug(12);

    // First Qddot is found
    // Stack The last Task
    for(int i(1); i<task_list.size(); ++i){
        task = task_list[i];

        if(!task->IsTaskSet()){ printf("%d th task is not set!\n", i); exit(0); }
        task->getTaskJacobian(Jt);
        task->getTaskJacobianDotQdot(JtDotQdot);
        task->getCommand(xddot);

        JtPre = Jt * Npre;
        _WeightedInverse(JtPre, Ainv_, JtPreBar);

        qddot_pre = qddot_pre + JtPreBar * (xddot - JtDotQdot - Jt * qddot_pre);

        Npre = Npre * (Eigen::MatrixXd::Identity(num_qdot_, num_qdot_)
                - JtPreBar * JtPre);
    }
    _PrintDebug(13);
    _GetSolution(qddot_pre, cmd);
    _PrintDebug(14);

    data_->opt_result_ = Eigen::VectorXd(dim_opt_);
    for(int i(0); i<dim_opt_; ++i){
        data_->opt_result_[i] = z[i];
    }
    //std::cout << "f: " << f << std::endl;
    //std::cout << "x: " << z << std::endl;
    //std::cout << "cmd: "<<cmd<<std::endl;
    //myUtils::pretty_print(Sf_, std::cout, "Sf");
    //myUtils::pretty_print(qddot_pre, std::cout, "qddot_pre");
    //myUtils::pretty_print(JcN, std::cout, "JcN");
    //myUtils::pretty_print(Nci_, std::cout, "Nci");
    //Eigen::VectorXd eq_check = dyn_CE * data_->opt_result_;
    //myUtils::pretty_print(dyn_ce0, std::cout, "dyn ce0");
    //myUtils::pretty_print(eq_check, std::cout, "eq_check");

    //myUtils::pretty_print(Jt, std::cout, "Jt");
    //myUtils::pretty_print(JtDotQdot, std::cout, "Jtdotqdot");
    //myUtils::pretty_print(xddot, std::cout, "xddot");


    //printf("G:\n");
    //std::cout<<G<<std::endl;
    //printf("g0:\n");
    //std::cout<<g0<<std::endl;

    //printf("CE:\n");
    //std::cout<<CE<<std::endl;
    //printf("ce0:\n");
    //std::cout<<ce0<<std::endl;

    //printf("CI:\n");
    //std::cout<<CI<<std::endl;
    //printf("ci0:\n");
    //std::cout<<ci0<<std::endl;


    // if(f > 1.e5){
    //   std::cout << "f: " << f << std::endl;
    //   std::cout << "x: " << z << std::endl;
    //   std::cout << "cmd: "<<cmd<<std::endl;

       //printf("G 1:\n");
       //std::cout<<G<<std::endl;
    //   printf("g0:\n");
    //   std::cout<<g0<<std::endl;

    //   printf("CE:\n");
    //   std::cout<<CE<<std::endl;
    //   printf("ce0:\n");
    //   std::cout<<ce0<<std::endl;

    //   printf("CI:\n");
    //   std::cout<<CI<<std::endl;
    //   printf("ci0:\n");
    //   std::cout<<ci0<<std::endl;
    // }

}

void WBLC::_SetInEqualityConstraint(){
    Eigen::MatrixXd dyn_CI(dim_ieq_cstr_, dim_opt_); dyn_CI.setZero();
    Eigen::VectorXd dyn_ci0(dim_ieq_cstr_);

    dyn_CI.block(0, dim_first_task_, dim_rf_cstr_, dim_rf_) = Uf_;
    dyn_ci0 = uf_ieq_vec_;

   for(int i(0); i< dim_ieq_cstr_; ++i){
        for(int j(0); j<dim_opt_; ++j){
            CI[j][i] = dyn_CI(i,j);
        }
        ci0[i] = -dyn_ci0[i];
    }
   // myUtils::pretty_print(dyn_CI, std::cout, "WBLC: CI");
    // myUtils::pretty_print(dyn_ci0, std::cout, "WBLC: ci0");
}

void WBLC::_ContactBuilding(const std::vector<WBLCContact*> & contact_list){
    Eigen::MatrixXd Uf;
    Eigen::VectorXd uf_ieq_vec;
    // Initial
    Eigen::MatrixXd Jc = contact_list[0]->getJc();
    Eigen::VectorXd JcDotQdot = contact_list[0]->getJcDotQDot();
    Jc_ = Jc;

    JcDotQdot_ = JcDotQdot;
    Uf_ = contact_list[0]->getWrenchFace();
    uf_ieq_vec_ = Eigen::VectorXd::Zero(Uf_.rows());

    dim_rf_ = Uf_.cols();
    dim_rf_cstr_ = Uf_.rows();

    int dim_new_rf, dim_new_rf_cstr;

    for(int i(1); i<contact_list.size(); ++i){
        Uf = contact_list[i]->getWrenchFace();
        uf_ieq_vec = Eigen::VectorXd::Zero(Uf_.rows());
        Jc = contact_list[i]->getJc();
        JcDotQdot = contact_list[i]->getJcDotQDot();
        dim_new_rf = Uf.cols();
        dim_new_rf_cstr = Uf.rows();

        // Jc append
        Jc_.conservativeResize(dim_rf_ + dim_new_rf, num_qdot_);
        Jc_.block(dim_rf_, 0, dim_new_rf, num_qdot_) = Jc;

        // JcDotQdot append
        JcDotQdot_.conservativeResize(dim_rf_ + dim_new_rf, 1);
        JcDotQdot_.tail(dim_new_rf) = JcDotQdot;

        // Uf
        Uf_.conservativeResize(dim_rf_cstr_ + dim_new_rf_cstr, dim_rf_ + dim_new_rf);
        Uf_.block(0, dim_rf_, dim_rf_cstr_, dim_new_rf).setZero();
        Uf_.block(dim_rf_cstr_, 0, dim_new_rf_cstr, dim_rf_).setZero();
        Uf_.block(dim_rf_cstr_, dim_rf_, dim_new_rf_cstr, dim_new_rf) = Uf;

        // Uf inequality vector
        uf_ieq_vec_.conservativeResize(dim_rf_cstr_ + dim_new_rf_cstr);
        uf_ieq_vec_.tail(dim_new_rf_cstr) = uf_ieq_vec;

        // Increase reaction force dimension
        dim_rf_ += dim_new_rf;
        dim_rf_cstr_ += dim_new_rf_cstr;
    }

    // printf("dim rf: %i, dim rf constr: %i \n", dim_rf_, dim_rf_cstr_);
    //JcDotQdot_.setZero();
    //myUtils::pretty_print(Jc_, std::cout, "WBLC: Jc");
    //myUtils::pretty_print(JcDotQdot_, std::cout, "WBLC: JcDot Qdot");
    //Eigen::MatrixXd Uf1 = Uf_.block(0, 0, 17, 6);
    //Eigen::MatrixXd Uf2 = Uf_.block(17, 6, 17, 6);
    //myUtils::pretty_print(Uf1, std::cout, "WBLC: Uf");
    //myUtils::pretty_print(Uf2, std::cout, "WBLC: Uf");
    //exit(0);
}

void WBLC::_GetSolution(const Eigen::VectorXd & qddot, Eigen::VectorXd & cmd){
    Eigen::VectorXd Fr(dim_rf_);
    for(int i(0); i<dim_rf_; ++i) Fr[i] = z[i + dim_first_task_];
    Eigen::VectorXd tot_tau = A_ * qddot + cori_ + grav_ - (Jc_* Nci_).transpose() * Fr;
    //myUtils::pretty_print(Fr, std::cout, "reaction force");

    Eigen::MatrixXd UNci = Sa_ * Nci_;
    Eigen::MatrixXd UNciBar;
    _WeightedInverse(UNci, Ainv_, UNciBar);
    cmd = UNciBar.transpose() * tot_tau;
    //myUtils::pretty_print(result, std::cout, "opt result");
    //myUtils::pretty_print(tot_tau, std::cout, "tot tau result");
    //myUtils::pretty_print(cmd, std::cout, "final command");
}

void WBLC::_OptimizationPreparation(){
    dim_opt_ = dim_rf_ + dim_first_task_;
    dim_eq_cstr_ = 6;
    dim_ieq_cstr_ = dim_rf_cstr_;

    G.resize(dim_opt_, dim_opt_);
    g0.resize(dim_opt_);
    CE.resize(dim_opt_, dim_eq_cstr_);
    ce0.resize(dim_eq_cstr_);
    CI.resize(dim_opt_, dim_ieq_cstr_);
    ci0.resize(dim_ieq_cstr_);

    for(int i(0); i<dim_opt_; ++i){
        for(int j(0); j<dim_opt_; ++j){
            G[i][j] = 0.;
        }
        g0[i] = 0.;
    }
    // Set Cost
    for (int i(0); i < dim_opt_; ++i){
        G[i][i] = data_->cost_weight[i];
    }
}

