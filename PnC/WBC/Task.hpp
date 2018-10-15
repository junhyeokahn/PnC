#ifndef WBC_TASK
#define WBC_TASK

#include <Eigen/Dense>

class RobotSystem;

enum TaskType { JOINT, LINKXYZ, LINKRPY, CENTROID, BASERPZ, COMRPY };

class Task{
    public:
        Task(RobotSystem* robot_, TaskType taskType_, std::string linkName_="");
        virtual ~Task();

        void getTaskJacobian(Eigen::MatrixXd & Jt_) { Jt_ = mJt; }
        void getTaskJacobianDotQdot(Eigen::VectorXd & JtDotQDot_) { JtDotQDot_ = mJtDotQDot; }

        void updateTaskSpec(const Eigen::VectorXd & pos_des,
                            const Eigen::VectorXd & vel_des,
                            const Eigen::VectorXd & acc_des);

        bool IsTaskSet() { return mIsUpdated; }
        int getDims() { return mDim; }
        TaskType getType() { return mTaskType; }
        void unsetTaskSpec() { mIsUpdated = false; }
        void setGain(const Eigen::VectorXd & kp_,
                     const Eigen::VectorXd & kd_);

        Eigen::VectorXd getAccCommand() { return mAccCmd; }
        Eigen::VectorXd getVelCommand() { return mVelCmd; }
        Eigen::VectorXd getPosDes() { return mPosDes; }
        Eigen::VectorXd getPosAcc() { return mPosAct; }
        Eigen::VectorXd getPosErr() { return mPosErr; }
        Eigen::VectorXd getVelDes() { return mVelDes; }
        Eigen::VectorXd getVelAcc() { return mVelAct; }
        Eigen::VectorXd getAccDes() { return mAccDes; }
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
         * BASERPZ :
         * pos_des_ = [x, w, z, w, z]
         * vel_des_ = [roll, pitch, zdot]
         * acc_des_ = [rolldot, pitchdot, zddot]
         *
         * COMRPY :
         * p -- center of mass position
         * pos_des_ = [px, py, pz, x, y, z, w]
         * vel_des_ = [pdotx, pdoty, pdotz, rdot, pdot, ydot]
         * acc_des_ = [pddotx, pddoty, pddotz, rddot, pddot, yddot]
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
                       const Eigen::VectorXd & acc_des_,
                       const Eigen::VectorXd & pos_,
                       const Eigen::VectorXd & vel_);

        TaskType mTaskType;
        std::string mLinkName;
        std::string mTypeString;
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
        Eigen::VectorXd mAccDes;
        Eigen::VectorXd mPosErr;
};

#endif
