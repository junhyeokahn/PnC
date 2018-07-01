#ifndef WBC_TASK
#define WBC_TASK

#include <Eigen/Dense>

class RobotSystem;

enum TaskType { JOINT, LINKXYZ, LINKRPY, COM };

class Task{
    public:
        Task(RobotSystem* robot_, TaskType taskType_, std::string linkName_="");
        virtual ~Task();

        void getCommand(Eigen::VectorXd & taskCmd_) { taskCmd_ = mTaskCmd; }
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
        // Update mTaskCmd
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
        Eigen::VectorXd mTaskCmd;
        Eigen::VectorXd mJtDotQDot;
        Eigen::MatrixXd mJt;

        bool mIsUpdated;
        int mDim;
        Eigen::VectorXd mKp;
        Eigen::VectorXd mKd;

        RobotSystem* mRobot;

        // Variables for DataManager
        Eigen::VectorXd mPosDes;
        Eigen::VectorXd mVelDes;
        Eigen::VectorXd mPosAct;
        Eigen::VectorXd mVelAct;
};

#endif
