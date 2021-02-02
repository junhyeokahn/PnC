#pragma once

#include <Utils/IO/IOUtilities.hpp>
#include <ExternalSource/myOptimizer/Goldfarb/QuadProg++.hh>
#include <PnC/WBC/WBC.hpp>
#include <PnC/WBC/ContactSpec.hpp>

class AWBC: public WBC{
    public:
        AWBC(const std::vector<bool>& act_list);
        virtual ~AWBC(){}

        virtual void updateJointSetting(const Eigen::VectorXd & q_cur, const Eigen::VectorXd & q_prev,
                                        const Eigen::VectorXd & dq_cur, const Eigen::VectorXd & dq_prev,
                                        const Eigen::VectorXd & q_des, const Eigen::VectorXd & dq_des);

        virtual void updateMassSetting(const Eigen::MatrixXd & A_cur,
                                       const Eigen::MatrixXd & A_prev, double & total_mass);

        virtual void updateCDSetting(const Eigen::Vector3d & pos_com_cur, const Eigen::Vector3d & pos_com_prev,
                                     const Eigen::MatrixXd & J_com_cur, const Eigen::MatrixXd & J_com_prev,
                                     const Eigen::MatrixXd & AM_cur, const Eigen::MatrixXd & AM_prev,
                                     const Eigen::MatrixXd & H_cur, const Eigen::MatrixXd & H_prev);

        virtual void updateContactSetting(const Eigen::VectorXd & Fr, const std::vector<Eigen::Vector3d> & contact_pos,  
                                          const Eigen::Vector3d & ext_pos, const Eigen::MatrixXd & J_ext_prev);

        void AdaptGains(const std::vector<ContactSpec*> & contact_list, Eigen::VectorXd & Kp_a, Eigen::VectorXd & Kd_a);

        void CheckDampingRatio(const Eigen::VectorXd & Kp, const Eigen::VectorXd & Kd, Eigen::VectorXd & Fs, Eigen::VectorXd & damping);                                            

        void EstimateExtforce(const std::vector<ContactSpec*> & contact_list);

        void setGains(const Eigen::VectorXd & Kp, const Eigen::VectorXd & Kd, const Eigen::VectorXd & Kp_h, const Eigen::VectorXd &Kd_h);

        Eigen::VectorXd hat_f_t_;
        Eigen::VectorXd hat_tau_t_;

    private:

        Eigen::Matrix3d _Skew(const Eigen::Vector3d r);

        bool b_Joint_;
        bool b_Mass_;
        bool b_CD_;
        bool b_contact_;

        double delta_t_;
        double total_m_;

        Eigen::VectorXd q_cur_;
        Eigen::VectorXd q_prev_;
        Eigen::VectorXd dq_cur_;
        Eigen::VectorXd dq_prev_;
        Eigen::VectorXd q_des_;
        Eigen::VectorXd dq_des_;

        Eigen::MatrixXd A_cur_;
        Eigen::MatrixXd A_prev_;
        Eigen::MatrixXd H_cur_;
        Eigen::MatrixXd H_prev_;

        Eigen::Vector3d pos_com_cur_;
        Eigen::Vector3d vel_com_cur_;
        Eigen::MatrixXd J_com_cur_;
        Eigen::MatrixXd AM_cur_;

        Eigen::Vector3d pos_com_prev_;
        Eigen::Vector3d vel_com_prev_;
        Eigen::MatrixXd J_com_prev_;
        Eigen::MatrixXd AM_prev_;

        Eigen::VectorXd Fr_;
        std::vector<Eigen::Vector3d> contact_pos_;
        Eigen::Vector3d ext_pos_;
        Eigen::MatrixXd J_ext_prev_;
        Eigen::VectorXd Ft_;

        Eigen::MatrixXd M_;

        Eigen::MatrixXd Sf_;
        Eigen::MatrixXd St_;
        Eigen::MatrixXd S_;

        Eigen::VectorXd hZq_; 
        Eigen::VectorXd hZdq_;

        Eigen::VectorXd g_;

        Eigen::VectorXd Kp_;
        Eigen::VectorXd Kd_;
        Eigen::VectorXd Kp_h_;
        Eigen::VectorXd Kd_h_;
        Eigen::VectorXd hat_Kp_;
        Eigen::VectorXd hat_Kd_;




};
