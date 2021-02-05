#include <Eigen/LU>
#include <Eigen/SVD>

#include <PnC/WBC/WBLC/AWBC.hpp>
#include <Utils/Math/pseudo_inverse.hpp>
#include <Utils/IO/IOUtilities.hpp>

AWBC::AWBC(const std::vector<bool>& act_list): WBC(act_list) {
    myUtils::pretty_constructor(3, "Adaptive WBC on");
    b_Joint_ = false;
    b_Mass_ = false;
    b_CD_ = false;
    b_contact_ = false;
    delta_t_ = 0.001;

    Sf_ = Eigen::MatrixXd::Zero(6,3);
    Sf_.block(3,0,3,3) = Eigen::MatrixXd::Identity(3,3);
    St_ = Eigen::MatrixXd::Zero(6,3);
    St_.block(0,0,3,3) = Eigen::MatrixXd::Identity(3,3);

    S_ = Eigen::MatrixXd::Zero(num_qdot_-6, num_qdot_);
    S_.block(0,6,num_qdot_-6,num_qdot_-6) = Eigen::MatrixXd::Identity(num_qdot_-6,num_qdot_-6);

    hZq_ = Eigen::VectorXd::Zero(num_qdot_);
    hZdq_ = Eigen::VectorXd::Zero(num_qdot_);

    g_ = Eigen::VectorXd::Zero(3);
    g_[2] = -9.81;

}

void AWBC::updateJointSetting(const Eigen::VectorXd & q_cur, const Eigen::VectorXd & q_prev,
                              const Eigen::VectorXd & dq_cur, const Eigen::VectorXd & dq_prev,
                              const Eigen::VectorXd & q_des, const Eigen::VectorXd & dq_des)
{
    q_cur_ = q_cur;
    q_prev_ = q_prev;
    dq_cur_ = dq_cur;
    dq_prev_ = dq_prev;
    q_des_ = q_des;
    dq_des_ = dq_des;

    b_Joint_ = true;
}

void AWBC::updateMassSetting(const Eigen::MatrixXd & A_cur,
                             const Eigen::MatrixXd & A_prev, double & total_mass)
{
    A_cur_ = A_cur;
    A_prev_ = A_prev;
    total_m_ = total_mass;
    b_Mass_ = true;
}

void AWBC::updateCDSetting(const Eigen::Vector3d & pos_com_cur, const Eigen::Vector3d & pos_com_prev,
                           const Eigen::MatrixXd & J_com_cur, const Eigen::MatrixXd & J_com_prev,
                           const Eigen::MatrixXd & AM_cur, const Eigen::MatrixXd & AM_prev,
                           const Eigen::MatrixXd & H_cur, const Eigen::MatrixXd & H_prev)
{
    pos_com_cur_ = pos_com_cur;
    pos_com_prev_ = pos_com_prev;
    J_com_cur_= J_com_cur;
    J_com_prev_ = J_com_prev;
    AM_cur_ = AM_cur;
    AM_prev_ = AM_prev;
    H_cur_ = H_cur;
    H_prev_ = H_prev;
    b_CD_ = true;
}

void AWBC::updateContactSetting(const Eigen::VectorXd & Fr, const std::vector<Eigen::Vector3d> & contact_pos, 
                                const Eigen::Vector3d & ext_pos, const Eigen::MatrixXd & J_ext_prev)
{
    Fr_ = Fr;
    contact_pos_ = contact_pos;
    ext_pos_ = ext_pos;
    J_ext_prev_ = J_ext_prev;
    b_contact_ = true;
}

void AWBC::setGains(const Eigen::VectorXd & Kp, const Eigen::VectorXd & Kd, const Eigen::VectorXd & Kp_h, const Eigen::VectorXd &Kd_h)
{
    Kp_ = Kp;
    Kd_ = Kd;
    Kp_h_ = Kp_h;
    Kd_h_ = Kd_h;

    hat_Kp_ = Kp_ + Kp_h_;
    hat_Kd_ = Kd_ + Kd_h_;
}

void AWBC::EstimateExtforce(const std::vector<ContactSpec*> & contact_list)
{
    if(!b_Joint_) printf("joint setting is not done\n");
    if(!b_Mass_) printf("mass setting is not done\n");
    if(!b_CD_) printf("centroidal setting is not done\n");
    if(!b_contact_) printf("contact setting is not done\n");

    // compute external force
    Eigen::VectorXd f_hat_q = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd c1 = Eigen::VectorXd::Zero(3);
    int num_contact = contact_list.size();
    int n = J_com_prev_.cols();

    Eigen::VectorXd sum_force = Eigen::VectorXd::Zero(3);
    for(int i(0); i< num_contact ; ++i)
        sum_force += Fr_.segment(6*i+3,3);

    f_hat_q = 2*total_m_/(delta_t_*delta_t_)*J_com_prev_.block(3,0,3,n)*q_cur_;

    c1 =  - 2*total_m_/(delta_t_*delta_t_)*J_com_prev_.block(3,0,3,n)*q_prev_ - sum_force 
         - total_m_*((1/delta_t_)*2.0*J_com_prev_.block(3,0,3,n)*dq_prev_ + g_);

    hat_f_t_ = f_hat_q + c1;
    
    // compute external torque
    Eigen::VectorXd tau_hat_q = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd tau_hat_dq = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd sum_torque = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd c2 = Eigen::VectorXd::Zero(3);

    Eigen::Vector3d temp;
    for(int i(0); i<3; ++i)
        temp[i] = f_hat_q[i];

    tau_hat_q =  (1/delta_t_)*H_prev_*q_cur_ - ext_pos_.cross(temp);
    tau_hat_dq = (1/delta_t_)*AM_prev_*dq_cur_;

    // myUtils::pretty_print(tau_hat_q, std::cout, "tau_hat_q");
    // myUtils::pretty_print(tau_hat_dq, std::cout, "tau_hat_dq");

    Eigen::Vector3d r;
    r.setZero();

    Eigen::MatrixXd temp_mat(3,n);
    Eigen::Matrix3d temp_skew;
    
    ContactSpec* contact = contact_list[0];
    Eigen::MatrixXd Jc(6,n);
    Eigen::MatrixXd Jc_stacked(3*num_contact,n);

    for(int i(0); i< num_contact ; ++i)
    {
        r = contact_pos_[i];
        for(int j(0); j<3; ++j)
            temp[j] = Fr_.segment(6*i+3,3)[j];

        sum_torque += r.cross(temp) + Fr_.segment(6*(i),3);
        
        temp_skew = _Skew(temp);
        contact = contact_list[i];
        contact->getContactJacobian(Jc);
        Jc_stacked.block(3*i,0,3,n) = Jc;
        temp_mat += temp_skew*Jc;
    }

    Eigen::MatrixXd Jc_stacked_inv;
    myUtils::weightedInverse(Jc_stacked, A_cur_.inverse(), Jc_stacked_inv);

    Eigen::MatrixXd Kp_mat = Eigen::MatrixXd::Zero(num_qdot_,num_qdot_);
    Kp_mat.block(6,6,num_qdot_-6,num_qdot_-6) = hat_Kp_.asDiagonal();
    Eigen::MatrixXd fc_coef = Jc_stacked_inv.transpose()*S_.transpose()*S_*A_cur_*Kp_mat;
    Eigen::MatrixXd sum_fc_coef = Eigen::MatrixXd::Zero(3,num_qdot_);

    for(int i(0); i< num_contact ; ++i)
        sum_fc_coef += fc_coef.block(3*i ,0, 3, num_qdot_);

    for(int i(0); i<3; ++i)
        temp[i] = c1[i];

    c2 = - (1/delta_t_)*(H_prev_*q_prev_ + AM_prev_*dq_prev_) - sum_torque - ext_pos_.cross(temp);

    // myUtils::pretty_print(sum_torque, std::cout, "sum_torque");
    // myUtils::pretty_print(c2, std::cout, "c2");
    // myUtils::pretty_print(temp_mat , std::cout, "temp_matrix");

    hat_tau_t_ = tau_hat_q + tau_hat_dq + c2; 

    Eigen::MatrixXd Kpf_tilde = Eigen::MatrixXd::Zero(n,n);
    Eigen::MatrixXd Kdf_tilde = Eigen::MatrixXd::Zero(n,n);

    double eta = 1/(delta_t_);

    for(int i(6); i< n; ++i){
        Kpf_tilde(i,i) = eta*(q_des_[i] - q_prev_[i]);
        // Kd_tilde(i,i) = ((2-delta_t_)*dq_cur_[i] - 2*dq_prev_[i])/(delta_t_*(dq_des_[i] - dq_cur_[i]));
        Kdf_tilde(i,i) =  - delta_t_*Kpf_tilde(i,i);
    }

    // myUtils::pretty_print(Kp_tilde, std::cout, "Kp_tilde");
    // myUtils::pretty_print(Kd_tilde, std::cout, "Kd_tilde");

    Eigen::MatrixXd Mp =  total_m_*J_com_prev_.block(3,0,3,n)*Kpf_tilde;
    Eigen::MatrixXd Md =  total_m_*J_com_prev_.block(3,0,3,n)*Kdf_tilde;

    // myUtils::pretty_print(Mp, std::cout, "Mp");
    // myUtils::pretty_print(Md, std::cout, "Md");

    Eigen::MatrixXd Kpt_tilde = Eigen::MatrixXd::Zero(n,n);
    Eigen::MatrixXd Kdt_tilde = Eigen::MatrixXd::Zero(n,n);

    for(int i(6); i< n; ++i){
        Kpt_tilde(i,i) = eta*(q_des_[i] - q_prev_[i]);
        // Kd_tilde(i,i) = ((2-delta_t_)*dq_cur_[i] - 2*dq_prev_[i])/(delta_t_*(dq_des_[i] - dq_cur_[i]));
        Kdt_tilde(i,i) =  - delta_t_*Kpt_tilde(i,i);
    }

    Eigen::Matrix3d rt_skew = _Skew(ext_pos_);
    Eigen::MatrixXd Np = H_prev_*Kpt_tilde - total_m_*rt_skew*J_com_prev_.block(3,0,3,n)*Kpf_tilde;
    Eigen::MatrixXd Nd = AM_prev_*Kdt_tilde - total_m_*rt_skew*J_com_prev_.block(3,0,3,n)*Kdf_tilde;

    myUtils::pretty_print(Np, std::cout, "Np");
    myUtils::pretty_print(Nd, std::cout, "Nd");
    myUtils::pretty_print(Np, std::cout, "H_prev_");
    myUtils::pretty_print(Nd, std::cout, "AM_prev_");

    Eigen::MatrixXd Zq_n = A_prev_.inverse()*J_ext_prev_.transpose()*(Sf_*Mp + St_*Np);
    Eigen::MatrixXd Zdq_n = A_prev_.inverse()*J_ext_prev_.transpose()*(Sf_*Md  + St_*Nd);

    // myUtils::pretty_print(Zq_n, std::cout, "Zq_n");
    // myUtils::pretty_print(Zdq_n, std::cout, "Zdq_n");

    hZq_ = Zq_n.diagonal();
    hZdq_ = Zdq_n.diagonal();

    // myUtils::pretty_print(hZq_, std::cout, "hZq");
    // myUtils::pretty_print(hZdq_, std::cout, "hZdq");
}

Eigen::Matrix3d AWBC::_Skew(const Eigen::Vector3d r)
{
    Eigen::Matrix3d Res;
    Res.setZero();

    Res(0,1) = - r[2];
    Res(0,2) = r[1];
    Res(1,2) = -r[0];
    Res(1,0) = -Res(0,1);
    Res(2,0) = -Res(0,2);
    return Res;
}

void AWBC::CheckDampingRatio(const Eigen::VectorXd & Kp, const Eigen::VectorXd & Kd, Eigen::VectorXd & Fs, Eigen::VectorXd & damping)
{
    double wn = 0.0;
    double zeta= 0.0;

    int n = Kp.size();
    Fs.resize(n);
    damping.resize(n);

    for(int i(0); i< Kp.size(); ++i){
        wn = sqrt(Kp[i]);
        zeta = Kd[i]/(2*wn);
        Fs[i] = wn;
        damping[i] = zeta;
    }
}

void AWBC::AdaptGains(const std::vector<ContactSpec*> & contact_list, Eigen::VectorXd & Kp_a, Eigen::VectorXd & Kd_a)
{
    Eigen::VectorXd Fs;
    Eigen::VectorXd damping;

    CheckDampingRatio(Kp_h_, Kd_h_, Fs, damping);
    // myUtils::pretty_print(Fs, std::cout, "Fs");
    EstimateExtforce(contact_list);

    Eigen::VectorXd epsilon_p = Eigen::VectorXd::Zero(Kp_.size());
    Eigen::VectorXd epsilon_d = Eigen::VectorXd::Zero(Kp_.size());
    Eigen::VectorXd zeta = Eigen::VectorXd::Zero(Kp_.size());
    
    double epsilon_limit = 100.;

    for(int i(0); i< Kp_.size(); ++i){
        epsilon_p[i] = - hZq_[6+i];

        if(epsilon_p[i] > epsilon_limit)
            epsilon_p[i] = epsilon_limit;
        else if(epsilon_p[i] < - epsilon_limit)
            epsilon_p[i] = - epsilon_limit;

        zeta[i] = hat_Kd_[i] / (2*sqrt(hat_Kp_[i]));
        epsilon_d[i] =  2*zeta[i]*sqrt(hat_Kp_[i] - hZq_[6+i] + epsilon_p[i]) - hat_Kd_[i] + hZdq_[6+i] ;

        if(Kd_h_[i] + epsilon_d[i] < 0.)
            epsilon_d[i] = -Kd_h_[i];
            
    }

    myUtils::pretty_print(epsilon_p, std::cout, "epsilon_p");
    myUtils::pretty_print(epsilon_d, std::cout, "epsilon_d");

    // check condition 3
    Eigen::VectorXd temp_vec = hat_Kp_ + epsilon_p - hZq_.segment(6, Kp_.size());
    double temp_val = temp_vec.sum();
    if(temp_val<0)
        printf("the 1st stability condition is invalid \n");
    
    // check condition 4
    temp_vec = hat_Kd_ + epsilon_d - hZdq_.segment(6, Kd_.size());
    temp_val = temp_vec.sum();
    if(temp_val<0)
        printf("the 2nd stability condition is invalid \n");

    Kp_a = Kp_h_ + epsilon_p;
    Kd_a = Kd_h_ + epsilon_d;

}