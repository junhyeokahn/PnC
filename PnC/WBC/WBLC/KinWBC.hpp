#pragma once

#include <vector>

#include <PnC/WBC/ContactSpec.hpp>
#include <PnC/WBC/Task.hpp>

class KinWBC {
    public:
        KinWBC(const std::vector<bool> & act_joint);
        ~KinWBC(){}

        bool FindConfiguration(
                const Eigen::VectorXd & curr_config,
                const std::vector<Task*> & task_list,
                const std::vector<ContactSpec*> & contact_list,
                Eigen::VectorXd & jpos_cmd,
                Eigen::VectorXd & jvel_cmd,
                Eigen::VectorXd & jacc_cmd);
        Eigen::MatrixXd Ainv_;

    private:
        void _PseudoInverse(const Eigen::MatrixXd J, Eigen::MatrixXd & Jinv);
        void _BuildProjectionMatrix(
                const Eigen::MatrixXd & J,
                Eigen::MatrixXd & N);

        double threshold_;
        int num_qdot_;
        int num_act_joint_;
        std::vector<int> act_jidx_;
        Eigen::MatrixXd I_mtx;
};
