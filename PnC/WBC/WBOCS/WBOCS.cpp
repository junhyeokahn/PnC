#include <PnC/RobotSystem/RobotSystem.hpp>
#include <PnC/WBC/WBOCS/WBOCS.hpp>
#include <Utils/Math/pseudo_inverse.hpp>
#include <Utils/IO/IOUtilities.hpp>

WBOCS::WBOCS(const std::vector<bool> & act_list, RobotSystem* robot): WBC(act_list) {
    myUtils::pretty_constructor(3, "Optimal Covraiance Steering on");
    robot_ = robot;
    N_receeding_ = 0;
    skel_ptr_ = robot_->getSkeleton();
    N_dof_ = robot_->getNumDofs();
}

WBOCS::setuplists(const std::vector<Task*> & task_list,
                  const std::vector<ContactSpec*> & contact_list){
    task_list_ = task_list;
    contact_list_ contact_list;
}

WBOCS::norminal_joint_reference(const std::vector<Eigen::VectorXd> & ref_q,
                                const std::vector<Eigen::VectorXd> & ref_dq,
                                const std::vector<Eigen::VectorXd> & ref_tau){
    ref_q_ = ref_q;
    ref_dq_ = ref_dq;
    ref_tau_ = ref_tau;

    N_recedding_ = ref_q_.size(); 
}



WBOCS::norminal_task_reference(const int idx_task,
                               std::vector<Eigen::Vector3d> & ref_pos,
                               std::vector<Eigen::Vector3d> & ref_vel,
                               std::vector<Eigen::vector3d> & ref_force){

    Eigen::VectorXd q_cur;
    Eigen::VectorXd dq_cur;

    Task* task = task_list[idx_task];

    for(int i(0; i< N_receeding, ++i){
        q_cur = ref_q_[i];
        dq_cur = ref_dq_[i];
        robot_->updateSystem(q_cur, dq_cur, true);
        

    }


}

WBOCS::updateTaskDynamics(const int idx_task,
                          const Eigen::MatrixXd & A_task,
                          const Eigen::MatrixXd & B_task,
                          const Eigen::MatrixXd & C_task,
                          const Eigen::VectorXd & r_task, double threshold){
    
    Eigen::MatrixXd A_t;
    Eigen::MatrixXd B_t;
    Eigen::MatrixXd C_t;
    Eigen::VectorXd r_t;

   
   

    



}


        
