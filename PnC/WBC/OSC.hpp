#pragma once

#include <Eigen/Dense>
#include <PnC/WBC/WBC.hpp>
#include <PnC/WBC/ContactSpec.hpp>
#include <Utils/IO/IOUtilities.hpp>

class Task;

class OSC: public WBC{
    public:
        OSC (const std::vector<bool> & act_list, const Eigen::MatrixXd* Jci = NULL);
        virtual ~OSC (){}
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

    private:
};
