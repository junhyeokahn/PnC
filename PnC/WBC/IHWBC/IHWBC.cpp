#include <PnC/WBC/IHWBC/IHWBC.hpp>

// Constructor
IHWBC::IHWBC(const std::vector<bool> & act_list): 
	num_act_joint_(0), num_passive_(0), 
	dim_contacts_(0),
	b_weights_set_(false),
	b_updatesetting_(false), 
	target_wrench_minimization(false) {
    myUtils::pretty_constructor(1, "IHWBC");
    // Set Number of degrees of freedom
    num_qdot_ = act_list.size();
    
    // Set number of active and passive joints
    for(int i(0); i<num_qdot_; ++i){
        if(act_list[i] == true){
            num_act_joint_++;  
        }else{
            num_passive_++;
        } 
    }

    // Selection Matrices
    Sa_ = Eigen::MatrixXd::Zero(num_act_joint_, num_qdot_);
    Sv_ = Eigen::MatrixXd::Zero(num_passive_, num_qdot_);
    Sf_ = Eigen::MatrixXd::Zero(6, num_qdot_);

    Jc_ = Eigen::MatrixXd::Zero(6, num_qdot_);

    // Set Floating Base Selection Matrices
    Sf_.block(0,0, 6, 6).setIdentity();

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

    // Set Tikhonov regularization for decision variables
    lambda_qddot = 1e-16;
	lambda_Fr = 1e-16;

}

// Destructor
IHWBC::~IHWBC(){}

// Returns the joint acceleration computed by the QP
void IHWBC::getQddotResult(Eigen::VectorXd & qddot_out){
    qddot_out = qddot_result_;
}
// Returns the reaction forces computed by the QP
void IHWBC::getFrResult(Eigen::VectorXd & Fr_out){
    Fr_out = Fr_result_;
}

void IHWBC::setQPWeights(const Eigen::VectorXd & w_task_heirarchy_in, 
                         const Eigen::VectorXd & w_rf_contacts_in, 
                         const double & w_contact_weight_in){
	w_task_heirarchy = w_task_heirarchy_in;
	w_rf_contacts = w_rf_contacts_in;
	w_contact_weight = w_contact_weight_in;
	b_weights_set_ = true;
}

void IHWBC::setQPWeights(const Eigen::VectorXd & w_task_heirarchy_in, const double & w_contact_weight_in){
	w_task_heirarchy = w_task_heirarchy_in;
	w_contact_weight = w_contact_weight_in;	
	b_weights_set_ = true;
}

void IHWBC::setRegularizationTerms(const double lambda_qddot_in, const double lambda_Fr_in){
	lambda_qddot = lambda_qddot_in;	
	lambda_Fr = lambda_Fr_in;
}

void IHWBC::setTargetWrenchMinimization(const bool target_wrench_minimization_in){
	target_wrench_minimization = target_wrench_minimization_in;
}

void IHWBC::updateSetting(const Eigen::MatrixXd & A,
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
void IHWBC::solve(const std::vector<Task*> & task_list,
		          const std::vector<ContactSpec*> & contact_list,
		          const Eigen::VectorXd & Fd,
		          Eigen::VectorXd & tau_cmd, Eigen::VectorXd & qddot_cmd){
    if(!b_updatesetting_) { printf("[Warning] IHWBC setting A, Ainv, cori, and grav terms have not been done\n"); }

	if(!b_weights_set_){
		printf("[Warning] Weights for IHBWC has not been set. Setting 1.0 to all weights \n");
		w_task_heirarchy = Eigen::VectorXd::Zero(task_list.size());
		w_rf_contacts = Eigen::VectorXd::Zero(contact_list.size());

		for(int i = 0; i < task_list.size(); i++){
			w_task_heirarchy[i] = 1.0;
		}
		for(int i = 0; i < contact_list.size(); i++){
			w_rf_contacts[i] = 1.0;
		}
		w_contact_weight = 1.0;
	}

    // Task Matrices and Vectors
    Eigen::MatrixXd Jt, Jc; 
    Eigen::VectorXd JtDotQdot, xddot;

    // Cost Matrices and Vectors
    Eigen::MatrixXd Pt, Pf; 
    Eigen::VectorXd vt, vf;

    // Task Cost Matrix
    Pt = Eigen::MatrixXd::Zero(num_qdot_, num_qdot_);
	vt = Eigen::VectorXd::Zero(num_qdot_);

	// Equivalent formulation of the task terms:
    // sum_i=1^n w_i|| J_i*qddot + J_i*dotqdot - x_iddot ||^2_2 + lambda_qddot*||qddot||^2_2
    Task* task;
    for(int i = 0; i < task_list.size(); i++){
    	task = task_list[i];
    	task->getTaskJacobian(Jt);
        task->getTaskJacobianDotQdot(JtDotQdot);
        task->getCommand(xddot);       

        // Add to Costs
        Pt += (w_task_heirarchy[i]*(Jt.transpose()*Jt));
        vt += (w_task_heirarchy[i]*((JtDotQdot-xddot).transpose()*Jt));
    }
    Pt += (lambda_qddot*Eigen::MatrixXd::Identity(num_qdot_, num_qdot_)); 

    // Prepare contact dimensions
    dim_contacts_ = 0;     
    dim_contact_constraints_ = 0;
    if (contact_list.size() > 0){
	    // Construct Contact Jacobians
	    buildContactStacks(contact_list, w_rf_contacts);    	

        if (!target_wrench_minimization){
    	    // w_f*||Fd - Fr||^2_2 + lambda_Fr*||Fr||
    	    // Target Force Minimization (term by term)

    	    // Force Contact Costs
    	    Pf = (w_contact_weight + lambda_Fr)*Eigen::MatrixXd::Identity(dim_contacts_, dim_contacts_);
    	    vf = -w_contact_weight*Fd.transpose()*Eigen::MatrixXd::Identity(dim_contacts_, dim_contacts_);    	
        }else{
    	    // Target Wrench Minimization
    	    // w_f*||Fw -  Sf* [wrf_1 *J^Tc_1, ..., wrf_n *J^Tc_n]*F_r_i|| + lambda_Fr*||Fr||
    	    // = w_f*||Fw -  Sf* [wrf_1 *Jc_1; ... ; wrf_n *J^c_n]^T*F_r_i|| + lambda_Fr*||Fr||
    	    // Count dimension of contacts from the contact_list

        	// Use Weighted Jacobian Stack
    		Eigen::MatrixXd Jf = Eigen::MatrixXd(6, num_qdot_);	
    		Jf = Sf_*(Jc_weighted_.transpose()); // Container to save computation

            myUtils::pretty_print(Jf, std::cout, "Jf");

    		// Force Contact Costs
    		Pf = w_contact_weight*(Jf.transpose()*Jf)  + lambda_Fr*Eigen::MatrixXd::Identity(dim_contacts_, dim_contacts_);
    		vf = -w_contact_weight*Fd.transpose()*(Jf);

            // Distribution matrix
            // Eigen::MatrixXd D_Fr(dim_contacts_, dim_contacts_); D_Fr.setIdentity();
            // int idx_offset = 0;
            // int dim_rf;
            // for(int i = 0; i < contact_list.size(); i++){
            //     dim_rf = contact_list[i]->getDim();
            //     D_Fr.block(idx_offset, idx_offset, dim_rf, dim_rf) = w_rf_contacts[i]*Eigen::MatrixXd::Identity(dim_rf, dim_rf);
            //     idx_offset += dim_rf;
            // }
            // myUtils::pretty_print(D_Fr, std::cout, "D_Fr");
            // Pf += (w_contact_weight*D_Fr);
            // vf += (-w_contact_weight*Fd.transpose()*(D_Fr));

        }
    }

    // Set total cost matrix and vector
    Eigen::MatrixXd Ptot(num_qdot_ + dim_contacts_, num_qdot_ + dim_contacts_);
    Eigen::VectorXd vtot(num_qdot_ + dim_contacts_); 
    Ptot.setZero(); vtot.setZero(); 
    
    // Assign Block Cost Matrices  
    Ptot.block(0, 0, num_qdot_, num_qdot_) = Pt;
    vtot.head(num_qdot_) = vt;
    if (dim_contacts_ > 0){
    	Ptot.block(num_qdot_, num_qdot_, dim_contacts_, dim_contacts_) = Pf;
    	vtot.tail(dim_contacts_) = vf;
    }

    dim_dec_vars_ = num_qdot_ + dim_contacts_;

    // Create the Dynamic Equality Constraint
    // CE^T = Sf(A - Jc^T )
    // ce0 = Sf(b + g)
	Eigen::MatrixXd dyn_CE(6, dim_dec_vars_);
	Eigen::VectorXd dyn_ce0(6);
    // Floating Base Dynamics
	dyn_CE.block(0, 0, 6, num_qdot_) = Sf_*A_;
	dyn_CE.block(0, num_qdot_, 6, dim_contacts_) = -Sf_*Jc_.transpose();
    dyn_ce0 = Sf_ * (cori_ + grav_);

    // Create the Dynamic Inequality Constraint
    dim_inequality_constraints_ = dim_contact_constraints_;
    Eigen::MatrixXd dyn_CI(dim_inequality_constraints_, dim_dec_vars_); 
    Eigen::VectorXd dyn_ci0(dim_inequality_constraints_);
    // Contact Constraints
    // U*Fr + *ci0*Fr >= 0   
    dyn_CI.setZero(); dyn_ci0.setZero();
    dyn_CI.block(0, num_qdot_, dim_contact_constraints_, dim_contacts_) = Uf_;
    dyn_ci0.segment(0, dim_contact_constraints_) = -uf_ieq_vec_;

    // myUtils::pretty_print(Uf_, std::cout, "Uf_");
    // myUtils::pretty_print(uf_ieq_vec_, std::cout, "uf_ieq_vec_");
    // myUtils::pretty_print(dyn_ci0, std::cout, "dyn_ci0");

    // To Do: Torque Constraints
    // tau_min <= Sa_(Aqddot - Jc_.transpose*Fr + cori_ + grav_ ) <= tau_max
    // Sa_(Aqddot - Jc_.transpose*Fr + cori_ + grav_ ) >= tau_min
    //   => Sa_(Aqddot - Jc_.transpose*Fr + cori_ + grav_ ) - tau_min >= 0
    // Sa_(Aqddot - Jc_.transpose*Fr + cori_ + grav_ ) <= tau_max    
    //   => -Sa_(Aqddot - Jc_.transpose*Fr + cori_ + grav_ ) + tau_max >= 0

    // To Do: Joint Limit Constraints

    // Solve Quadprog
    prepareQPSizes(); // Prepare QP size
    setQuadProgCosts(Ptot, vtot); // Set Quadprog Costs
    setEqualityConstraints(dyn_CE, dyn_ce0); // Create Equality Constraints
    setInequalityConstraints(dyn_CI, dyn_ci0); // Create Inequality Constraints
    solveQP();

    if (contact_list.size() > 0){
        tau_cmd = Sa_*(A_*qddot_result_ + cori_ + grav_ - Jc_.transpose()*Fr_result_);        
    }else{
        tau_cmd = Sa_*(A_*qddot_result_ + cori_ + grav_);                
    }

    qddot_cmd = Sa_*qddot_result_;

}

// Creates a stack of contact jacobians that are weighted by w_rf_contacts
// Also create the contact constraints
void IHWBC::buildContactStacks(const std::vector<ContactSpec*> & contact_list, const Eigen::VectorXd & w_rf_contacts_in){
    // Temporary containers 
    // Contact Constraints
    Eigen::MatrixXd Uf;
    Eigen::VectorXd uf_ieq_vec;
    // Set First Contact Constraint
    contact_list[0]->getRFConstraintMtx(Uf_);
    contact_list[0]->getRFConstraintVec(uf_ieq_vec_);

    // Contact Jacobian
    Eigen::MatrixXd Jc;	
	contact_list[0]->getContactJacobian(Jc);

	Jc_ = Jc;
	if (target_wrench_minimization){
		Jc_weighted_ = w_rf_contacts_in[0]*Jc;		
	}

    int dim_rf = contact_list[0]->getDim();
    int dim_rf_cstr = contact_list[0]->getDimRFConstratint();
    int dim_new_rf, dim_new_rf_cstr;

    for(int i(1); i<contact_list.size(); ++i){
        contact_list[i]->getContactJacobian(Jc);
        dim_new_rf = contact_list[i]->getDim();
        dim_new_rf_cstr = contact_list[i]->getDimRFConstratint();

        // Stack Jc normally
        Jc_.conservativeResize(dim_rf + dim_new_rf, num_qdot_);
        Jc_.block(dim_rf, 0, dim_new_rf, num_qdot_) = Jc;

        // Jc stack with relative weighting
        if (target_wrench_minimization){
	        Jc_weighted_.conservativeResize(dim_rf + dim_new_rf, num_qdot_);
	        Jc_weighted_.block(dim_rf, 0, dim_new_rf, num_qdot_) = w_rf_contacts_in[i]*Jc;
        }

        // Uf
        contact_list[i]->getRFConstraintMtx(Uf);
        Uf_.conservativeResize(dim_rf_cstr + dim_new_rf_cstr, dim_rf + dim_new_rf);
        Uf_.block(0, dim_rf, dim_rf_cstr, dim_new_rf).setZero();
        Uf_.block(dim_rf_cstr, 0, dim_new_rf_cstr, dim_rf).setZero();
        Uf_.block(dim_rf_cstr, dim_rf, dim_new_rf_cstr, dim_new_rf) = Uf;

        // Uf inequality vector
        contact_list[i]->getRFConstraintVec(uf_ieq_vec);
        uf_ieq_vec_.conservativeResize(dim_rf_cstr + dim_new_rf_cstr);
        uf_ieq_vec_.tail(dim_new_rf_cstr) = uf_ieq_vec; 

        // Increase reaction force dimension
        dim_rf += dim_new_rf;
        dim_rf_cstr += dim_new_rf_cstr;
    }

    dim_contacts_ = dim_rf;
    dim_contact_constraints_ = dim_rf_cstr;
}

void IHWBC::prepareQPSizes(){
	n_quadprog_ = dim_dec_vars_; // Number of decision Variables
	p_quadprog_ = dim_inequality_constraints_;	 // Number of Inequality constraints
	m_quadprog_ = 6; 	// Number of Equality Constraints

	qp_dec_vars_ = Eigen::VectorXd::Zero(n_quadprog_);
	qddot_result_ = Eigen::VectorXd::Zero(num_qdot_);
	Fr_result_ = Eigen::VectorXd::Zero(dim_contacts_);

	// Decision Variables
	x.resize(n_quadprog_);
	// Objective
	G.resize(n_quadprog_, n_quadprog_);
	g0.resize(n_quadprog_);
	// Equality Constraints
	CE.resize(n_quadprog_, m_quadprog_);
	ce0.resize(m_quadprog_);
	// Inequality Constraints
	CI.resize(n_quadprog_, p_quadprog_);  
	ci0.resize(p_quadprog_);
}

void IHWBC::setQuadProgCosts(const Eigen::MatrixXd & P_cost, const Eigen::VectorXd & v_cost){
	// Set G
	for(int i = 0; i < n_quadprog_; i++){
		for(int j = 0; j < n_quadprog_; j++){
			G[i][j] = P_cost(i,j);
		}
	}
	// Set g0
	for(int i = 0; i < n_quadprog_; i++){
		g0[i] = v_cost[i];
	}
}

void IHWBC::setEqualityConstraints(const Eigen::MatrixXd & Eq_mat, const Eigen::VectorXd & Eq_vec){
  for(int i = 0; i < m_quadprog_; i++){
    for(int j = 0; j < n_quadprog_; j++){
      CE[j][i] = Eq_mat(i,j);
    }
    ce0[i] = Eq_vec[i];
  }

}

void IHWBC::setInequalityConstraints(const Eigen::MatrixXd & IEq_mat, const Eigen::VectorXd & IEq_vec){
    for (int i = 0; i < p_quadprog_; ++i) {
      for (int j = 0; j < n_quadprog_; ++j) {
          CI[j][i] = IEq_mat(i, j);
      }
      ci0[i] = IEq_vec[i];
    }
}

void IHWBC::solveQP(){
	double qp_result = solve_quadprog(G, g0, CE, ce0, CI, ci0, x);

	// Populate qd result from QP answer
	for(int i = 0; i < n_quadprog_; i++){
		qp_dec_vars_[i] = x[i];
	}
	// Store results
	qddot_result_ = qp_dec_vars_.head(num_qdot_);
	Fr_result_ = qp_dec_vars_.tail(dim_contacts_);

	// myUtils::pretty_print(qp_dec_vars_, std::cout, "qp_dec_vars_");
	// myUtils::pretty_print(qddot_result_, std::cout, "qddot_result_");
	// myUtils::pretty_print(Fr_result_, std::cout, "Fr_result_");

}

/*

Eigen::MatrixXd dyn_CE(dim_eq_cstr_, dim_opt_);
Eigen::VectorXd dyn_ce0(dim_eq_cstr_);

Aqddot + b + g = Jc^T Fr

Sf(Aqddot + b + g - Jc^T Fr = 0)

Sf*A*qddot + Sf*(b+g) - (Sf*Jc^T) Fr = 0)

Sf*A*qddot + Sf*(b+g) - (Sf*Jc^T) Fr = 0)

[Sf*A - Sf*Jc^T] [qddot, Fr] = 0


*/