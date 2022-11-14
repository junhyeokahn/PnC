//
// Created by carlos on 11/9/22.
//
#include <gtest/gtest.h>
#include <utils/util.hpp>
#include <cmath>

#include <pnc/planners/locomotion/dcm_planner/dcm_planner.hpp>
#include <pnc/whole_body_controllers/managers/dcm_trajectory_manager.hpp>
#include "pnc/robot_system/dart_robot_system.hpp"
#include "pnc/draco_pnc/draco_tci_container.hpp"

// for plotting purposes only
//#include <matplotlibcpp.h>
#include "plot/sciplot/sciplot.hpp"

const double error_tolerance = 1.0e-3;

using namespace sciplot;

TEST(DCMPlannerTest, inPlaceStepping)
{
  bool b_plot_trajectories = true;

  YAML::Node cfg = YAML::LoadFile(THIS_COM "config/draco/pnc.yaml");
  auto servo_dt = util::ReadParameter<double>(cfg, "servo_dt");

  auto robot = new DartRobotSystem(THIS_COM "robot_model/draco/draco_rel_path.urdf",
                               false, false);
  auto tci_container = new DracoTCIContainer(robot);
  auto dcm_planner = new DCMPlanner();

  auto dcm_tm = new DCMTrajectoryManager(dcm_planner,
                          tci_container->com_task, tci_container->torso_ori_task,
                          robot, "l_foot_contact", "r_foot_contact");

  dcm_tm->paramInitialization(cfg["walking"]);

  // at contact_transition_start
  int transfer_type = dcm_transfer_type::kInitial;

  double ini_time = 40.54;
  double curr_time = ini_time;
  Eigen::Vector3d ini_pos = Eigen::Vector3d(0.1185, 0.0272, 0.);
  Eigen::Vector3d ini_vel = Eigen::Vector3d::Zero();
  Eigen::Quaternion<double> torso_quat;
  Eigen::Vector3d act_torso_rpy = Eigen::Vector3d(-0.001, -0.005, 1.8);
  Eigen::Vector3d des_torso_rpy = Eigen::Vector3d(0.0, 0.0, 1.79);
  torso_quat = util::EulerZYXtoQuat(act_torso_rpy(0), act_torso_rpy(1), act_torso_rpy(2));

  dcm_tm->walkInPlace();
  Footstep l_footstep = Footstep();
  Footstep r_footstep = Footstep();
  Footstep mid_footstep = Footstep();
  auto lfoot_ori = Eigen::Quaterniond (util::EulerZYXtoQuat(-0.004, -0.026, 1.792));
  auto rfoot_ori = Eigen::Quaterniond (util::EulerZYXtoQuat(-0.075, -0.05, 1.80)); // actual
  auto lfoot_pos = Eigen::Vector3d(0., 0., 0.);
  auto rfoot_pos = Eigen::Vector3d(0.2325, 0.052, 0.002);
  l_footstep.setPosOriSide(lfoot_pos, lfoot_ori, EndEffector::LFoot);
  r_footstep.setPosOriSide(rfoot_pos, rfoot_ori, EndEffector::RFoot);
  mid_footstep.computeMidfeet(l_footstep, r_footstep, mid_footstep);
  dcm_tm->left_foot_stance_ = l_footstep;
  dcm_tm->right_foot_stance_ = r_footstep;
  dcm_tm->mid_foot_stance_ = mid_footstep;
  dcm_tm->initialize(curr_time, transfer_type, torso_quat, ini_pos, ini_vel);

  dcm_tm->updateDCMTasksDesired(curr_time);
//  std::cout << "rvrp[0]" << dcm_planner->rvrp_list[0].transpose() << std::endl;
//  std::cout << "rvrp[1]" << dcm_planner->rvrp_list[1].transpose() << std::endl;
//  std::cout << "rvrp[2]" << dcm_planner->rvrp_list[2].transpose() << std::endl;
//  std::cout << "rvrp[3]" << dcm_planner->rvrp_list[3].transpose() << std::endl;

//  std::cout << "dcm_end_DS_list[0]" << dcm_planner->dcm_end_DS_list[0].transpose() << std::endl;
//  std::cout << "dcm_end_DS_list[1]" << dcm_planner->dcm_end_DS_list[1].transpose() << std::endl;
//  std::cout << "dcm_end_DS_list[2]" << dcm_planner->dcm_end_DS_list[2].transpose() << std::endl;
//  std::cout << "dcm_end_DS_list[3]" << dcm_planner->dcm_end_DS_list[3].transpose() << std::endl;

  Eigen::Vector3d des_dcm;
  std::vector<double> des_dcm_traj_x, des_dcm_traj_y, des_dcm_traj_z;
  int n_steps = dcm_tm->n_steps;
  while(curr_time < ini_time + dcm_planner->get_t_step_end(n_steps+1))
  {
    curr_time += servo_dt;
    dcm_tm->updateDCMTasksDesired(curr_time);
    dcm_planner->get_ref_dcm(curr_time, des_dcm);
    des_dcm_traj_x.push_back(des_dcm(0));
    des_dcm_traj_y.push_back(des_dcm(1));
    des_dcm_traj_z.push_back(des_dcm(2));
    std::cout << "curr_time:" << curr_time << ", des_dcm" << des_dcm.transpose() << std::endl;
  }

  if (b_plot_trajectories)
  {
    Plot2D plot;
    plot.xlabel("x");
    plot.ylabel("y");
    plot.drawCurve(des_dcm_traj_x, des_dcm_traj_y).label("des dcm");

    // Create figure to hold plot
    Figure fig = {{plot}};
    // Create canvas to hold figure
    Canvas canvas = {{fig}};

    // Show the plot in a pop-up window
    canvas.show();
  }

  Eigen::Vector3d dcm_pre_end = dcm_planner->dcm_end_DS_list[n_steps];
  Eigen::Vector3d dcm_end = dcm_planner->dcm_end_DS_list[n_steps+1];
  ASSERT_FALSE((des_dcm - dcm_pre_end).norm() < error_tolerance);
  ASSERT_TRUE((des_dcm - dcm_end).norm() < error_tolerance);

}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}