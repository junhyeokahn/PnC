#include <Eigen/Dense>

#include <PnC/WBC/Task.hpp>

enum BasicTaskType { JOINT, LINKXYZ, LINKRPY, CENTROID };

class BasicTask : public Task {
    public:
        BasicTask (RobotSystem* robot_,
                   const BasicTaskType & taskType_,
                   const int & _dim,
                   const std::string & linkName_="");
        virtual ~BasicTask() {};

    private:
        /* Update mAccCmd, mVelCmd
         *
         * CENTROID :
         *  p -- center of mass position
         *  k -- angular momentum
         *  l -- linear momentum
         *  pos_des_ = [dummy , dummy , dummy ,  px   , py    , pz     ]
         *  vel_des_ = [kx    , ky    , kz    ,  lx   , ly    , lz     ]
         *  acc_des_ = [kx_dot, ky_dot, kz_dot, lx_dot, ly_dot, lz_dot ]
         *
         */
        virtual bool _UpdateCommand(const Eigen::VectorXd & pos_des,
                                    const Eigen::VectorXd & vel_des,
                                    const Eigen::VectorXd & acc_des);
        virtual bool _UpdateTaskJacobian();
        virtual bool _UpdateTaskJDotQdot();

        BasicTaskType task_type_;
        std::string link_name_;
        std::string task_type_string_;
};
