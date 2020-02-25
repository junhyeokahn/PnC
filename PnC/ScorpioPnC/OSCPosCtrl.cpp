#include <PnC/ScorpioPnC/OSCPosCtrl.hpp>
#include <PnC/ScorpioPnC/ScorpioInterface.hpp>
#include <PnC/ScorpioPnC/ScorpioDefinition.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>

OSCPosCtrl::OSCPosCtrl(RobotSystem* _robot) : Controller(_robot) {
    myUtils::pretty_constructor(2, "OSC POS Ctrl");
    ctrl_count_ = 0;
    active_joint_idx_.resize(Scorpio::n_adof);
    active_joint_idx_[0] = 0;
    active_joint_idx_[1] = 1;
    active_joint_idx_[2] = 4;
    active_joint_idx_[3] = 5;
    active_joint_idx_[4] = 8;
    active_joint_idx_[5] = 9;
    active_joint_idx_[6] = 10;
}

OSCPosCtrl::~OSCPosCtrl() {}

void OSCPosCtrl::oneStep(void* _cmd) {
    Eigen::VectorXd gamma = Eigen::VectorXd::Zero(Scorpio::n_adof);
    //construct selection matrix
    Eigen::MatrixXd S= Eigen::MatrixXd::Zero(Scorpio::n_adof,robot_->getNumDofs());
    std::vector<bool> act_list;
    act_list.resize(Scorpio::n_dof,true);
    act_list[2] = false;
    act_list[3] = false;
    act_list[6] = false;
    act_list[7] = false;
    int j(0);
    int i(0);
    for (int i = 0; i < Scorpio::n_dof; ++i) {
       if(act_list[i]){ 
           S(j,i) = 1.;
            ++j;
       }
    }
    //myUtils::pretty_print(S, std::cout, "selection matrix");

    Eigen::VectorXd end_effector_pos_des = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd end_effector_vel_des = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd end_effector_acc_ff = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd end_effector_acc_des = Eigen::VectorXd::Zero(3);

    Eigen::VectorXd end_effector_pos_act = robot_->getBodyNodeIsometry("end_effector").translation();
    Eigen::VectorXd end_effector_vel_act = robot_->getBodyNodeSpatialVelocity("end_effector").tail(3); 

    for (int i = 0; i < 3; ++i) {
            end_effector_pos_des[i] = myUtils::smooth_changing(ini_pos_[i], ini_pos_[i] + target_pos_[i],
                                                             end_time_, state_machine_time_);
            end_effector_vel_des[i] = myUtils::smooth_changing_vel(ini_pos_[i], ini_pos_[i] + target_pos_[i],
                                                                    end_time_, state_machine_time_);
            end_effector_acc_ff[i] = myUtils::smooth_changing_acc(ini_pos_[i], ini_pos_[i] + target_pos_[i],
                                                                    end_time_, state_machine_time_);
            end_effector_acc_des[i] =
                end_effector_acc_ff[i] + end_effector_kp_[i] * (end_effector_pos_des[i] - end_effector_pos_act[i]) 
                + end_effector_kd_[i] * (end_effector_vel_des[i] - end_effector_vel_act[i]);

            //end_effector_acc_des[i] =
                  //end_effector_kp_[i] * (end_effector_pos_des[i] - end_effector_pos_act[i]) 
                //- end_effector_kd_[i] * (end_effector_vel_act[i]);
        }
    Eigen::MatrixXd J_constraints = Eigen::MatrixXd::Zero(6,Scorpio::n_dof);

    //dart::dynamics::SkeletonPtr skel_ptr = robot_-> getSkeleton();
    //dart::dynamics::BodyNodePtr bodynode1 = skel_ptr -> getBodyNode("link2");
    //Eigen::Vector3d null = Eigen::Vector3d::Zero();
    //Eigen::MatrixXd J_con = robot_->getBodyNodeJacobian("link4_end",null,bodynode1).block(3,0,3,robot_->getNumDofs());

    Eigen::MatrixXd J_body_1 = robot_->getBodyNodeJacobian("link2").block(3,0,3,robot_->getNumDofs()); 
    Eigen::MatrixXd J_constriant_1 = robot_->getBodyNodeJacobian("link4_end").block(3,0,3,robot_->getNumDofs());
    Eigen::MatrixXd J_constriant_diff_1 = J_constriant_1 - J_body_1;
    J_constraints.block(0,0,3,robot_->getNumDofs()) =  J_constriant_diff_1;

    //myUtils::pretty_print(J_con, std::cout, "11111");
    //myUtils::pretty_print(J_constriant_diff_1, std::cout, "22222");
 
    Eigen::MatrixXd J_body_2 = robot_->getBodyNodeJacobian("link6").block(3,0,3,robot_->getNumDofs()); 
    Eigen::MatrixXd J_constriant_2 = robot_->getBodyNodeJacobian("link8_end").block(3,0,3,robot_->getNumDofs());
    Eigen::MatrixXd J_constriant_diff_2 = J_constriant_2 - J_body_2;
    J_constraints.block(3,0,3,robot_->getNumDofs()) =  J_constriant_diff_2;
    //myUtils::pretty_print(J_constraints, std::cout, "J_C");

    //Eigen::VectorXd vel1 = J_constriant_1* robot_->getQdot();
    //Eigen::VectorXd vel2 = J_constriant_2* robot_->getQdot();

    //Eigen::VectorXd vel1_global = J_constriant_diff_1* robot_->getQdot();
    //Eigen::VectorXd vel2_global = J_constriant_diff_2* robot_->getQdot();

    //myUtils::pretty_print(vel1, std::cout, "velocity 01");
    //myUtils::pretty_print(vel2, std::cout, "velocity 02");

    //myUtils::pretty_print(vel1_global, std::cout, "velocity 01-1");
    //myUtils::pretty_print(vel2_global, std::cout, "velocity 02-1");

    //Eigen::JacobiSVD<Eigen::MatrixXd> svd1(
            //J_constraints, Eigen::ComputeThinU | Eigen::ComputeThinV);
    //std::cout << "J_c singularValues" << std::endl;
    //std::cout << svd1.singularValues() << std::endl;
    //std::cout << "============================" << std::endl;

    Eigen::MatrixXd J_constraints_bar = Eigen::MatrixXd::Zero(Scorpio::n_dof,6);
    myUtils::weightedInverse(J_constraints,robot_->getInvMassMatrix(),J_constraints_bar);
    Eigen::MatrixXd N_c = Eigen::MatrixXd::Identity(Scorpio::n_dof,Scorpio::n_dof) - J_constraints_bar * J_constraints;
    //myUtils::pretty_print(N_c, std::cout, "N_c");

     //Eigen::JacobiSVD<Eigen::MatrixXd> svd2(
     //N_c, Eigen::ComputeThinU | Eigen::ComputeThinV);
     //std::cout << "N_c singularValues" << std::endl;
     //std::cout << svd2.singularValues() << std::endl;
     //std::cout << "============================" << std::endl;

    Eigen::MatrixXd b_c = N_c.transpose() * (robot_->getCoriolisGravity());

    Eigen::MatrixXd SN_c = S * N_c;
    //PRINT
     //Eigen::JacobiSVD<Eigen::MatrixXd> svd3(
     //SN_c, Eigen::ComputeThinU | Eigen::ComputeThinV);
     //std::cout << "SN_c singularValues" << std::endl;
     //std::cout << svd3.singularValues() << std::endl;
     //std::cout << "============================" << std::endl;
     //PRINT
    Eigen::MatrixXd SN_c_bar = Eigen::MatrixXd::Zero(Scorpio::n_dof,Scorpio::n_adof);
    myUtils::weightedInverse(SN_c, robot_->getInvMassMatrix(), SN_c_bar);

    //Eigen::JacobiSVD<Eigen::MatrixXd> svd4(
    //SN_c_bar, Eigen::ComputeThinU | Eigen::ComputeThinV);
    //std::cout << "SN_c_bar singularValues" << std::endl;
    //std::cout << svd4.singularValues() << std::endl;
    //std::cout << "============================" << std::endl;
 
    Eigen::MatrixXd J_end_effector = robot_->getBodyNodeJacobian("end_effector").block(3,0,3,robot_->getNumDofs());
    Eigen::MatrixXd J_end_effector_bar = Eigen::MatrixXd::Zero(robot_->getNumDofs(),3);
    myUtils::weightedInverse(J_end_effector,robot_->getInvMassMatrix(),J_end_effector_bar);
    Eigen::MatrixXd N_end_effector = Eigen::MatrixXd::Identity(Scorpio::n_dof,Scorpio::n_dof) - J_end_effector_bar * J_end_effector; 
    Eigen::MatrixXd J_end_effector_dot = robot_->getBodyNodeJacobianDot("end_effector").block(3,0,3,robot_->getNumDofs());
    Eigen::VectorXd qddot_des_end_effector = Eigen::VectorXd::Zero(robot_->getNumDofs());

    qddot_des_end_effector = J_end_effector_bar * (end_effector_acc_des - J_end_effector_dot*(robot_->getQdot()));

   Eigen::MatrixXd J_q = S;      
   Eigen::VectorXd qddot_des_q = Eigen::VectorXd::Zero(Scorpio::n_dof); 
   Eigen::VectorXd joint_des_acc = Eigen::VectorXd::Zero(Scorpio::n_adof);
   Eigen::MatrixXd J_q_N_end_effector = J_q * N_end_effector;
   Eigen::MatrixXd J_q_N_end_effector_bar = Eigen::MatrixXd::Zero(Scorpio::n_dof,Scorpio::n_adof);
    myUtils::weightedInverse(J_q_N_end_effector,robot_->getInvMassMatrix(),J_q_N_end_effector_bar);

   for (int i = 0; i < Scorpio::n_adof; ++i) {
      joint_des_acc[i] = q_kp_[i] * (ini_pos_q[active_joint_idx_[i]] - robot_->getQ()[active_joint_idx_[i]])
                        + q_kd_[i] * (ini_vel_q[active_joint_idx_[i]] - robot_->getQdot()[active_joint_idx_[i]]);
   }

    qddot_des_q = J_q_N_end_effector_bar * (joint_des_acc - J_q * qddot_des_end_effector); 

    gamma = SN_c_bar.transpose() *(robot_->getMassMatrix() * (qddot_des_end_effector + qddot_des_q) + b_c);

    ((ScorpioCommand*)_cmd)->jtrq = gamma;

    ++ctrl_count_;
    state_machine_time_= ctrl_count_ * ScorpioAux::ServoRate;

}

void OSCPosCtrl::firstVisit() {
    state_machine_time_= 0.;
    ctrl_count_ = 0;
    ini_pos_ = robot_->getBodyNodeIsometry("end_effector").translation();
    ini_vel_ = robot_->getBodyNodeSpatialVelocity("end_effector").tail(3); 
    //std::cout << "initial end effector pos" << std::endl;
    //std::cout << ini_pos_ << std::endl;
    ini_pos_q = robot_->getQ();
    ini_vel_q = robot_->getQdot();
}
void OSCPosCtrl::lastVisit() {
}

bool OSCPosCtrl::endOfPhase() {
    if (state_machine_time_ > end_time_) {
            return true;
        }
    return false;
}

void OSCPosCtrl::ctrlInitialization(const YAML::Node& node) {
    try {
            myUtils::readParameter(node, "q_kp", q_kp_);
            myUtils::readParameter(node, "q_kd", q_kd_);
            myUtils::readParameter(node, "end_effector_kp", end_effector_kp_);
            myUtils::readParameter(node, "end_effector_kd", end_effector_kd_);
        } catch (std::runtime_error& e) {
                std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                          << __FILE__ << "]" << std::endl
                          << std::endl;
                exit(0);
            }
}
