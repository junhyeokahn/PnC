
#include <Eigen/LU>
#include <Eigen/SVD>

#include <PnC/WBC/WBIC.hpp>
#include <Utils/IO/IOUtilities.hpp>

WBIC::WBIC(const std::vector<bool> & act_list, const std::vector<Task*> task_list_, const std::vector<ContactSpec*> contact_list_) {
    myUtils::pretty_constructor(3, "WBIC");

    // Set number of active and passive joints
    num_act_joint_ = 12;
    num_passive_ = 6;

    Sa_ = Eigen::MatrixXd::Zero(num_act_joint_, num_qdot_);
    Sv_ = Eigen::MatrixXd::Zero(num_passive_, num_qdot_);
    Sa_.block(0, num_passive_, num_act_joint_, num_qdot_).setIdentity();
    Sv_.block(0, 0, num_passive_, num_passive_).setIdentity();

    _contact_list = contact_list_;
    _task_list = task_list_;

    _eye = Eigen::MatrixXd::Identity(num_qdot_, num_qdot_);
    _eye_floating = Eigen::MatrixXd::Identity(num_passive_, num_passive_);
}


void WBIC::updateSetting(const Eigen::MatrixXd& A, const Eigen::MatrixXd& Ainv,
                         const Eigen::VectorXd& cori,
                         const Eigen::VectorXd& grav, void* extra_setting) {
    A_ = A;
    Ainv_ = Ainv;
    cori_ = cori;
    grav_ = grav;
    b_updatesetting_ = true;

    (void)extra_setting;

    // dynacore::pretty_print(grav_, std::cout, "grav");
    // dynacore::pretty_print(cori_, std::cout, "cori");
    // dynacore::pretty_print(A_, std::cout, "A");
}

void WBIC::makeTorque(const std::vector<Task*> & task_list,
                      const std::vector<ContactSpec*> & contact_list,
                      Eigen::VectorXd& cmd, void* extra_input) {
    if(!b_updatesetting_){
        printf("[Warning] WBIC setting is not done\n");
    }
    if(extra_input) _data = static_cast<WBIC_ExtraData*>(extra_input);

    _SetOptimizationSize();
    _SetCost();

    Eigen::VectorXd qddot_pre;
    Eigen::MatrixXd JcBar;
    Eigen::MatrixXd Npre;

    if (_dim_rf > 0) {
    // Contact Setting
    _ContactBuilding();

    // Set inequality constraints
    _SetInequalityConstraint();
    myUtils::weightedInverse(_Jc, Ainv_, JcBar);
    qddot_pre = JcBar * (-_JcDotQdot);
    Npre = _eye - JcBar * _Jc;
    // myUtils::pretty_print(JcBar, std::cout, "JcBar");
    // myUtils::pretty_print(_JcDotQdot, std::cout, "JcDotQdot");
    // myUtils::pretty_print(qddot_pre, std::cout, "qddot 1");
  } else {
    qddot_pre = Eigen::VectorXd::Zero(num_qdot_);
    Npre = _eye;
  }

    Task* task;
    Eigen::MatrixXd Jt, JtBar, JtPre;
    Eigen::VectorXd JtDotQdot, xddot;

    for(int i=0; i< _task_list.size(); ++i){
        task = _task_list[i];
        task->getTaskJacobian(Jt);
        task->getTaskJacobianDotQdot(JtDotQdot);
        task->getCommand(xddot);

        JtPre = Jt * Npre;
        myUtils::weightedInverse(JtPre, Ainv_, JtBar);

        qddot_pre += JtBar * (xddot - JtDotQdot - Jt * qddot_pre);
        Npre = Npre * (_eye - JtBar * JtPre);

        myUtils::pretty_print(xddot, std::cout, "xddot");
        myUtils::pretty_print(JtDotQdot, std::cout, "JtDotQdot");
        myUtils::pretty_print(qddot_pre, std::cout, "qddot 2");
        myUtils::pretty_print(Jt, std::cout, "Jt");
        myUtils::pretty_print(JtPre, std::cout, "JtPre");
        myUtils::pretty_print(JtBar, std::cout, "JtBar");
    }

    // Set Eq Constraints
    _SetEqualityConstraint(qddot_pre);

    double f = solve_quadprog(G, g0, CE, ce0, CI, ci0, z);

    for (int i(0); i < 6; ++i) {qddot_pre[i] += z[i];}
    
    _GetSolution(qddot_pre, cmd);

    _data->_opt_result = Eigen::VectorXd(_dim_opt);

    for(int i=0; i< _dim_opt; ++i) {
        _data->_opt_result[i] = z[i];
    }

    // std::cout << "f: " << f << std::endl;
    //std::cout << "x: " << z << std::endl;

    // Eigen::VectorXd check_eq = _dyn_CE * _data->_opt_result + _dyn_ce0;
    // myUtils::pretty_print(check_eq, std::cout, "equality constr");
    // std::cout << "cmd: "<<cmd<<std::endl;
    // myUtils::pretty_print(qddot_pre, std::cout, "qddot_pre");
    // myUtils::pretty_print(JcN, std::cout, "JcN");
    // myUtils::pretty_print(Nci_, std::cout, "Nci");
    // Eigen::VectorXd eq_check = dyn_CE * data_->opt_result_;
    // myUtils::pretty_print(dyn_ce0, std::cout, "dyn ce0");
    // myUtils::pretty_print(eq_check, std::cout, "eq_check");

    // myUtils::pretty_print(Jt, std::cout, "Jt");
    // myUtils::pretty_print(JtDotQdot, std::cout, "Jtdotqdot");
    // myUtils::pretty_print(xddot, std::cout, "xddot");
    // printf("CE:\n");
    // std::cout<<CE<<std::endl;
    // printf("ce0:\n");
    // std::cout<<ce0<<std::endl;

    // printf("CI:\n");
    // std::cout<<CI<<std::endl;
    // printf("ci0:\n");
    // std::cout<<ci0<<std::endl;
}

void WBIC::_SetEqualityConstraint(const Eigen::VectorXd & qddot) {
	if (_dim_rf > 0) {
    	_dyn_CE.block(0, 0, _dim_eq_cstr, 6) =
      		A_.block(0, 0, 6, 6);
    	_dyn_CE.block(0, 6, _dim_eq_cstr, _dim_rf) =
      		-Sv_ * _Jc.transpose();
    	_dyn_ce0 = -Sv_ * (A_ * qddot + cori_ + grav_ -
        	_Jc.transpose() * _Fr_des);
  } else {
    	_dyn_CE.block(0, 0, _dim_eq_cstr, 6) =
      		A_.block(0, 0, 6, 6);
    	_dyn_ce0 = -Sv_ * (A_ * qddot + cori_ + grav_);
  }

  for (int i(0); i < _dim_eq_cstr; ++i) {
    for (int j(0); j < _dim_opt; ++j) {
      CE[j][i] = _dyn_CE(i, j);
    }
    ce0[i] = -_dyn_ce0[i];
  }
  // myUtils::pretty_print(_dyn_CE, std::cout, "WBIC: CE");
  // myUtils::pretty_print(_dyn_ce0, std::cout, "WBIC: ce0");
}

void WBIC::_SetInequalityConstraint() {
	_dyn_CI.block(0, num_passive_, _dim_Uf, _dim_rf) = _Uf;
  	_dyn_ci0 = _Uf_ieq_vec - _Uf * _Fr_des;

  	for (int i(0); i < _dim_Uf; ++i) {
    	for (int j(0); j < _dim_opt; ++j) {
      		CI[j][i] = _dyn_CI(i, j);
    	}
    	ci0[i] = -_dyn_ci0[i];
  	}
  // myUtils::pretty_print(_dyn_CI, std::cout, "WBIC: CI");
  // myUtils::pretty_print(_dyn_ci0, std::cout, "WBIC: ci0");

}

void WBIC::_ContactBuilding() {
  Eigen::MatrixXd Uf;
  Eigen::VectorXd Uf_ieq_vec;
  // Initial
  Eigen::MatrixXd Jc;
  Eigen::VectorXd JcDotQdot;
  int dim_accumul_rf, dim_accumul_uf;
  _contact_list[0]->getContactJacobian(Jc);
  _contact_list[0]->getJcDotQdot(JcDotQdot);
  _contact_list[0]->getRFConstraintMtx(Uf);
  _contact_list[0]->getRFConstraintVec(Uf_ieq_vec);

  dim_accumul_rf = _contact_list[0]->getDim();
  dim_accumul_uf = _contact_list[0]->getDimRFConstraint();

  _Jc.block(0, 0, dim_accumul_rf, num_qdot_) = Jc;
  _JcDotQdot.head(dim_accumul_rf) = JcDotQdot;
  _Uf.block(0, 0, dim_accumul_uf, dim_accumul_rf) = Uf;
  _Uf_ieq_vec.head(dim_accumul_uf) = Uf_ieq_vec;
  _Fr_des.head(dim_accumul_rf) = _contact_list[0]->getRFDesired();

  int dim_new_rf, dim_new_uf;

  for (int i(1); i < _contact_list.size(); ++i) {
    _contact_list[i]->getContactJacobian(Jc);
    _contact_list[i]->getJcDotQdot(JcDotQdot);

    dim_new_rf = _contact_list[i]->getDim();
    dim_new_uf = _contact_list[i]->getDimRFConstraint();

    // Jc append
    _Jc.block(dim_accumul_rf, 0, dim_new_rf, num_qdot_) = Jc;

    // JcDotQdot append
    _JcDotQdot.segment(dim_accumul_rf, dim_new_rf) = JcDotQdot;

    // Uf
    _contact_list[i]->getRFConstraintMtx(Uf);
    _Uf.block(dim_accumul_uf, dim_accumul_rf, dim_new_uf, dim_new_rf) = Uf;

    // Uf inequality vector
    _contact_list[i]->getRFConstraintVec(Uf_ieq_vec);
    _Uf_ieq_vec.segment(dim_accumul_uf, dim_new_uf) = Uf_ieq_vec;

    // Fr desired
    _Fr_des.segment(dim_accumul_rf, dim_new_rf) =
      _contact_list[i]->getRFDesired();
    dim_accumul_rf += dim_new_rf;
    dim_accumul_uf += dim_new_uf;
  }
  // myUtils::pretty_print(_Fr_des, std::cout, "[WBIC] Fr des");
  // myUtils::pretty_print(_Jc, std::cout, "[WBIC] Jc");
  // myUtils::pretty_print(_JcDotQdot, std::cout, "[WBIC] JcDot Qdot");
  // myUtils::pretty_print(_Uf, std::cout, "[WBIC] Uf");

}

void WBIC::_GetSolution(const Eigen::VectorXd & qddot, Eigen::VectorXd& cmd) {
  Eigen::VectorXd tot_tau;
  if (_dim_rf > 0) {
    _data->_Fr = Eigen::VectorXd(_dim_rf);
    // get Reaction forces
    for (int i(0); i < _dim_rf; ++i)
      _data->_Fr[i] = z[i + 6] + _Fr_des[i];
    tot_tau =
      A_ * qddot + cori_ + grav_ - _Jc.transpose() * _data->_Fr;

  } else {
    tot_tau = A_ * qddot + cori_ + grav_;
  }
  _data->_qddot = qddot;
  cmd = tot_tau.tail(12);

  // Torque check
  // DVec<T> delta_tau = DVec<T>::Zero(WB::num_qdot_);
  // for(int i(0); i<_dim_floating; ++i) delta_tau[i] = z[i];
  // myUtils::pretty_print(tot_tau, std::cout, "tot tau original");
  // tot_tau += delta_tau;
  // myUtils::pretty_print(tot_tau, std::cout, "tot tau result");
  // myUtils::pretty_print(qddot, std::cout, "qddot");
  // myUtils::pretty_print(_data->_Fr, std::cout, "Fr");
  // myUtils::pretty_print(_Fr_des, std::cout, "Fr des");t_;
    // myUtils::pretty_print(x_check, std::cout, "x check");
}


void WBIC::_SetCost() {
  // Set Cost
  int idx_offset(0);
  for (int i(0); i < 6; ++i) {
    G[i + idx_offset][i + idx_offset] = _data->_W_floating[i];
  }
  idx_offset += _dim_floating;
  for (int i(0); i < _dim_rf; ++i) {
    G[i + idx_offset][i + idx_offset] = _data->_W_rf[i];
  }
  // myUtils::pretty_print(_data->_W_floating, std::cout, "W floating");
  // myUtils::pretty_print(_data->_W_rf, std::cout, "W rf");
}

void WBIC::_SetOptimizationSize() {
  // Dimension
  _dim_rf = 0;
  _dim_Uf = 0;  // Dimension of inequality constraint
  for (size_t i(0); i < _contact_list.size(); ++i) {
    _dim_rf += _contact_list[i]->getDim();
    _dim_Uf += _contact_list[i]->getDimRFConstraint();
  }

  _dim_opt = 6 + _dim_rf;
  _dim_eq_cstr = 6;

  // Matrix Setting
  G.resize(0., _dim_opt, _dim_opt);
  g0.resize(0., _dim_opt);
  CE.resize(0., _dim_opt, _dim_eq_cstr);
  ce0.resize(0., _dim_eq_cstr);

  // Eigen Matrix Setting
  _dyn_CE = Eigen::MatrixXd::Zero(_dim_eq_cstr, _dim_opt);
  _dyn_ce0 = Eigen::VectorXd(_dim_eq_cstr);
  if (_dim_rf > 0) {
    CI.resize(0., _dim_opt, _dim_Uf);
    ci0.resize(0., _dim_Uf);
    _dyn_CI = Eigen::MatrixXd::Zero(_dim_Uf, _dim_opt);
    _dyn_ci0 = Eigen::VectorXd(_dim_Uf);

    _Jc = Eigen::MatrixXd(_dim_rf, num_qdot_);
    _JcDotQdot = Eigen::VectorXd(_dim_rf);
    _Fr_des = Eigen::VectorXd(_dim_rf);

    _Uf = Eigen::MatrixXd(_dim_Uf, _dim_rf);
    _Uf.setZero();
    _Uf_ieq_vec = Eigen::VectorXd(_dim_Uf);
  } else {
    CI.resize(0., _dim_opt, 1);
    ci0.resize(0., 1);
  }	
}
