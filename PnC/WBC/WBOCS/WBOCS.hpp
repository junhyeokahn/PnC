#pragma once

#include <Utils/IO/IOUtilities.hpp>
#include <ExternalSource/myOptimizer/Goldfarb/QuadProg++.hh>
#include <PnC/WBC/WBC.hpp>
#include <PnC/WBC/ContactSpec.hpp>

class RobotSystem;

class WBOCS: public WBC{
    public:
        WBLC(const std::vector<bool> & act_list, RobotSystem* robot, const Eigen::MatrixXd* Jci = NULL);
        virtual ~WBLC(){}

        virtual void updateSetting(const Eigen::MatrixXd & A,
                                   const Eigen::MatrixXd & Ainv,
                                   const Eigen::VectorXd & cori,
                                   const Eigen::VectorXd & grav,
                                   void* extra_setting = NULL);

        void setuplists(const std::vector<Task*> & task_list,
                        const std::vector<ContactSpec*> & contact_list);
        
        void norminal_joint_reference(const std::vector<Eigen::VectorXd> & ref_q,
                                      const std::vector<Eigen::VectorXd> & ref_dq,
                                      const std::vector<Eigen::VectorXd> & ref_tau);

        void norminal_task_reference(const int idx_task,
                                     std::vector<Eigen::Vector3d> & ref_pos,
                                     std::vector<Eigen::Vector3d> & ref_vel,
                                     std::vector<Eigen::vector3d> & ref_force);

        void  updateTaskDynamics(const int idx_task,
                                 const Eigen::MatrixXd & A_task,
                                 const Eigen::MatrixXd & B_task,
                                 const Eigen::MatrixXd & C_task,
                                 const Eigen::VectorXd & r_task, double threshold);

        void setconditions(const double mu_init, const Eigen::MatrixXd & cov_init,
                           const double mu_fin, const Eigen::MatrixXd & cov_fin);
        
    private:
        RobotSystem* robot_;
        dart::dynamics::SkeletonPtr skel_ptr_;
        std::vector<Task*> task_list_;
        std::vector<ContactSpec*> contact_list_;

        std::vector<Eigen::Vector3d> ref_pos_;
        std::vector<Eigen::Vector3d> ref_vel_;
        std::vector<Eigen::vector3d> ref_force_;

        std::vector<Eigen::Vector3d> ref_q_;
        std::vector<Eigen::Vector3d> ref_dq_;
        std::vector<Eigen::vector3d> ref_tau_;

        double mu_init_;
        double mu_fin_;

        Eigen::MatrixXd cov_init_;
        Eigen::MatrixXd cov_fin_;

        int N_receeding_;
        int N_dof_;

};
