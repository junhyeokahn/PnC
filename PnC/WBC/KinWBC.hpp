// #ifndef KINEMATICS_WHOLE_BODY_CONTROL
// #define KINEMATICS_WHOLE_BODY_CONTROL

#include <WBC/ContactSpec.hpp>
#include <WBC/Task.hpp>
#include <vector>


class KinWBC {
 public:
  KinWBC(size_t num_qdot);
  ~KinWBC() {}

  bool FindConfiguration(const Eigen::VectorXd & curr_config,
                         const std::vector<Task*> & task_list,
                         const std::vector<ContactSpec*> & contact_list,
                         Eigen::VectorXd & jpos_cmd, Eigen::VectorXd & jvel_cmd);

  Eigen::MatrixXd Ainv_;

 private:
  void _PseudoInverse(const Eigen::MatrixXd J, Eigen::MatrixXd & Jinv);
  void _BuildProjectionMatrix(const Eigen::MatrixXd & J, Eigen::MatrixXd & N);

  double threshold_;
  size_t num_qdot_;
  size_t num_act_joint_;
  Eigen::MatrixXd I_mtx;
};
#endif
