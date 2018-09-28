#ifndef WBC_TASK
#define WBC_TASK

#include <Eigen/Dense>

class RobotSystem;

enum TaskType { JOINT, LINKXYZ, LINKRPY, CENTROID };

class Task{
    public:
        Task(RobotSystem* robot_, TaskType taskType_, std::string linkName_="");
        virtual ~Task();

        void getAccCommand(Eigen::VectorXd & accCmd_) { accCmd_ = mAccCmd; }
        void getVelCommand(Eigen::VectorXd & velCmd_) { velCmd_ = mVelCmd; }
        void getTaskJacobian(Eigen::MatrixXd & Jt_) { Jt_ = mJt; }
        void getTaskJacobianDotQdot(Eigen::VectorXd & JtDotQDot_) { JtDotQDot_ = mJtDotQDot; }

        void updateTaskSpec(const Eigen::VectorXd & pos_des,
                            const Eigen::VectorXd & vel_des,
                            const Eigen::VectorXd & acc_des);

        bool IsTaskSet() { return mIsUpdated; }
        int getDims() { return mDim; }
        void unsetTaskSpec() { mIsUpdated = false; }
        void setGain(const Eigen::VectorXd & kp_,
                     const Eigen::VectorXd & kd_);

    private:
        /* Update mAccCmd, mVelCmd
         * CENTROID :
         *  p -- center of mass position
         *  k -- angular momentum
         *  l -- linear momentum
         *  pos_des_ = [dummy , dummy , dummy ,  px   , py    , pz     ]
         *  vel_des_ = [kx    , ky    , kz    ,  lx   , ly    , lz     ]
         *  acc_des_ = [kx_dot, ky_dot, kz_dot, lx_dot, ly_dot, lz_dot ]
         */
        void _updateCommand(const Eigen::VectorXd & pos_des,
                            const Eigen::VectorXd & vel_des,
                            const Eigen::VectorXd & acc_des);
        // Update mJt
        void _updateJt();
        // Update mJtDotQDot
        void _updateJtDotQDot();
        void _saveTask(const Eigen::VectorXd & pos_des_,
                       const Eigen::VectorXd & vel_des_,
                       const Eigen::VectorXd & pos_,
                       const Eigen::VectorXd & vel_);

        TaskType mTaskType;
        std::string mType;
        std::string mLinkName;
        Eigen::VectorXd mAccCmd;
        Eigen::VectorXd mVelCmd;
        Eigen::VectorXd mJtDotQDot;
        Eigen::MatrixXd mJt;

        bool mIsUpdated;
        int mDim;
        Eigen::VectorXd mKp;
        Eigen::VectorXd mKd;
        Eigen::VectorXd mKi;
        Eigen::VectorXd mErrSum;

        RobotSystem* mRobot;

        // Variables for DataManager
        Eigen::VectorXd mPosDes;
        Eigen::VectorXd mVelDes;
        Eigen::VectorXd mPosAct;
        Eigen::VectorXd mVelAct;
};

#endif
