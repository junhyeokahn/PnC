#pragma once

#include <Eigen/Dense>

#include <PnC/WBC/Task.hpp>

enum BasicTaskType {
  JOINT,
  LINKXYZ,
  LINKORI,
  ISOLATED_LINKXYZ,
  ISOLATED_LINKORI,
  CENTROID,
  COM
};

class BasicTask : public Task {
 public:
  BasicTask(RobotSystem* robot_, const BasicTaskType& taskType_,
            const int& _dim, const int& _link_idx = 0);
  virtual ~BasicTask(){};

  int getLinkID() { return link_idx_; }

 private:
  /* Update op_cmd, pos_err, vel_des, acc_des
   *
   * LINKORI :
   * pos_des_ = [quat_w, quat_x, quat_y, quat_z]
   * vel_des_ = [w_x, w_y, w_z]
   * acc_des_ = [a_x, a_y, a_z]
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
  virtual bool _UpdateCommand(const Eigen::VectorXd& pos_des,
                              const Eigen::VectorXd& vel_des,
                              const Eigen::VectorXd& acc_des);
  virtual bool _UpdateCurrent();
  virtual bool _UpdateTaskJacobian();
  virtual bool _UpdateTaskJDotQdot();

  BasicTaskType task_type_;
  int link_idx_;
  std::string task_type_string_;
};
