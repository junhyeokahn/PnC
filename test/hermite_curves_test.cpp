#include <gtest/gtest.h>

#include "utils/util.hpp"
#include "utils/interpolation/hermite_curve_vec.hpp"
#include "utils/interpolation/hermite_curve.hpp"
#include "utils/interpolation/cubic_hermite_curve.hpp"
#include "utils/interpolation/quintic_hermite_curve.hpp"
#include "utils/interpolation/min_jerk_curve.hpp"

#include "third_party/sciplot/sciplot.hpp"

static double err_tol = 1e-3;

using namespace sciplot;

TEST(HermiteCurveTest, compareCubicAndQuinticSplines) {
  bool b_plot = false;

  Eigen::Vector3d start_pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d start_vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d start_acc = Eigen::Vector3d::Zero();
  Eigen::Vector3d end_pos = 5. * Eigen::Vector3d::Ones();
  Eigen::Vector3d end_vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d end_acc = Eigen::Vector3d::Zero();
  double duration = 1.0;
  double dt = 0.001;
  CubicHermiteCurve cubicSpline = CubicHermiteCurve(start_pos[0], start_vel[0],
                                                    end_pos[0], end_vel[0], duration);
  double testSplineEnd = cubicSpline.evaluate(duration);
  ASSERT_TRUE(std::abs(testSplineEnd - end_pos[0]) <= err_tol);

  HermiteCurveVec cubicSplineVec = HermiteCurveVec(start_pos.size());
  HermiteCurveVec quinticSplineVec = HermiteCurveVec(start_pos.size());

  // construct vector of cubic and quintic splines to compare
  for (int i = 0; i < start_pos.size(); i++) {
    cubicSplineVec.add_curve(std::make_unique<CubicHermiteCurve>(
        start_pos[i], start_vel[i], end_pos[i], end_vel[i], duration));
    quinticSplineVec.add_curve(std::make_unique<QuinticHermiteCurve>(
        start_pos[i], start_vel[i], start_acc[i], end_pos[i], end_vel[i], end_acc[i], duration));
  }

  Eigen::Vector3d pos_des_cub, vel_des_cub, acc_des_cub;
  Eigen::Vector3d pos_des_quint, vel_des_quint, acc_des_quint;

  // variables to plot
  std::vector<double> pos_qunitic_traj_x, pos_quintic_traj_y, pos_quintic_traj_z, t_traj;
  std::vector<double> pos_cubic_traj_x, pos_cubic_traj_y, pos_cubic_traj_z;
  std::vector<double> vel_qunitic_traj_x, vel_quintic_traj_y, vel_quintic_traj_z;
  std::vector<double> vel_cubic_traj_x, vel_cubic_traj_y, vel_cubic_traj_z;
  std::vector<double> acc_qunitic_traj_x, acc_quintic_traj_y, acc_quintic_traj_z;
  std::vector<double> acc_cubic_traj_x, acc_cubic_traj_y, acc_cubic_traj_z;
  double t = 0.;
  while(t < duration)
  {
    pos_des_cub = cubicSplineVec.evaluate(t);
    vel_des_cub = cubicSplineVec.evaluateFirstDerivative(t);
    acc_des_cub = cubicSplineVec.evaluateSecondDerivative(t);

    pos_des_quint = quinticSplineVec.evaluate(t);
    vel_des_quint = quinticSplineVec.evaluateFirstDerivative(t);
    acc_des_quint = quinticSplineVec.evaluateSecondDerivative(t);

    // save positions
    pos_qunitic_traj_x.push_back(pos_des_quint[0]);
    pos_quintic_traj_y.push_back(pos_des_quint[1]);
    pos_quintic_traj_z.push_back(pos_des_quint[2]);
    pos_cubic_traj_x.push_back(pos_des_cub[0]);
    pos_cubic_traj_y.push_back(pos_des_cub[1]);
    pos_cubic_traj_z.push_back(pos_des_cub[2]);

    // save velocities
    vel_qunitic_traj_x.push_back(vel_des_quint[0]);
    vel_quintic_traj_y.push_back(vel_des_quint[1]);
    vel_quintic_traj_z.push_back(vel_des_quint[2]);
    vel_cubic_traj_x.push_back(vel_des_cub[0]);
    vel_cubic_traj_y.push_back(vel_des_cub[1]);
    vel_cubic_traj_z.push_back(vel_des_cub[2]);

    // save accelerations
    acc_qunitic_traj_x.push_back(acc_des_quint[0]);
    acc_quintic_traj_y.push_back(acc_des_quint[1]);
    acc_quintic_traj_z.push_back(acc_des_quint[2]);
    acc_cubic_traj_x.push_back(acc_des_cub[0]);
    acc_cubic_traj_y.push_back(acc_des_cub[1]);
    acc_cubic_traj_z.push_back(acc_des_cub[2]);

    t_traj.push_back(t);
    t += dt;
  }

  if(b_plot)
  {
    Plot2D plot_z_pos;
    plot_z_pos.xlabel("t");
    plot_z_pos.ylabel("p_z");
    plot_z_pos.drawCurve(t_traj, pos_quintic_traj_z).label("quintic spline");
    plot_z_pos.drawCurve(t_traj, pos_cubic_traj_z).label("cubic spline");
    plot_z_pos.legend().atTopRight();

    Plot2D plot_z_vel;
    plot_z_vel.xlabel("t");
    plot_z_vel.ylabel("v_z");
    plot_z_vel.drawCurve(t_traj, vel_quintic_traj_z).label("quintic spline");
    plot_z_vel.drawCurve(t_traj, vel_cubic_traj_z).label("cubic spline");
    plot_z_vel.legend().atTopRight();

    Plot2D plot_z_acc;
    plot_z_acc.xlabel("t");
    plot_z_acc.ylabel("a_z");
    plot_z_acc.drawCurve(t_traj, acc_quintic_traj_z).label("quintic spline");
    plot_z_acc.drawCurve(t_traj, acc_cubic_traj_z).label("cubic spline");
    plot_z_acc.legend().atTopRight();

    plot_z_pos.grid();

    // Create figure to hold plot_z_pos, vel, acc (vertically)
    Figure fig = {{plot_z_pos}, {plot_z_vel}, {plot_z_acc}};
    // Create canvas to hold figure
    Canvas canvas = {{fig}};

    canvas.size(750, 750);

    // Show the plot_z_pos in a pop-up window
    canvas.show();
  }

  // check end of trajectory
  for (unsigned int i = 0; i < start_pos.size(); i++) {
    ASSERT_TRUE(std::abs(pos_des_cub[i] - end_pos[i]) <= err_tol);
    ASSERT_TRUE(std::abs(pos_des_quint[i] - end_pos[i]) <= err_tol);
  }

  duration = duration / 4.0;
  pos_des_cub = cubicSplineVec.evaluate(duration);
  vel_des_cub = cubicSplineVec.evaluateFirstDerivative(duration);
  acc_des_cub = cubicSplineVec.evaluateSecondDerivative(duration);
  pos_des_quint = quinticSplineVec.evaluate(duration);
  vel_des_quint = quinticSplineVec.evaluateFirstDerivative(duration);
  acc_des_quint = quinticSplineVec.evaluateSecondDerivative(duration);
  // check mid-trajectories are not the same for cubic and quintic splines
  for (unsigned int i = 0; i < start_pos.size(); i++) {
    ASSERT_TRUE(std::abs(pos_des_quint[i] - pos_des_cub[i]) > err_tol);
    ASSERT_TRUE(std::abs(vel_des_quint[i] - vel_des_cub[i]) > err_tol);
    ASSERT_TRUE(std::abs(acc_des_quint[i] - acc_des_cub[i]) > err_tol);
  }
}

TEST(HermiteCurveTest, checkMinJerkTerminalConstraints) {
  bool b_plot = false;

  Eigen::Vector3d start_pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d start_vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d start_acc = Eigen::Vector3d::Zero();
  Eigen::Vector3d end_pos = 5. * Eigen::Vector3d::Ones();
  Eigen::Vector3d end_vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d end_acc = Eigen::Vector3d::Zero();
  double duration = 1.0;
  double dt = 0.001;

  HermiteCurveVec minJerkCurve = HermiteCurveVec(start_pos.size());

  // construct vector of min jerk trajectories
  Eigen::Vector3d init_state, end_state;
  for (int i = 0; i < start_pos.size(); i++) {
    init_state.setZero();
    init_state << start_pos[i], start_vel[i], start_acc[i];
    end_state << end_pos[i], end_vel[i], end_acc[i];
    minJerkCurve.add_curve(std::make_unique<MinJerkCurve>(
            init_state, end_state, 0.0, duration));
  }

  Eigen::Vector3d pos_des_minj, vel_des_minj, acc_des_minj;

  // variables to plot
  std::vector<double> pos_minj_traj_x, pos_minj_traj_y, pos_minj_traj_z, t_traj;
  std::vector<double> vel_minj_traj_x, vel_minj_traj_y, vel_minj_traj_z;
  std::vector<double> acc_minj_traj_x, acc_minj_traj_y, acc_minj_traj_z;
  double t = 0.;

  pos_des_minj = minJerkCurve.evaluate(t);
  vel_des_minj = minJerkCurve.evaluateFirstDerivative(t);
  acc_des_minj = minJerkCurve.evaluateSecondDerivative(t);
  // check end of trajectory
  for (unsigned int i = 0; i < start_pos.size(); i++) {
    ASSERT_TRUE(std::abs(pos_des_minj[i] - start_pos[i]) <= err_tol);
  }

  t = duration;
  pos_des_minj = minJerkCurve.evaluate(t);
  vel_des_minj = minJerkCurve.evaluateFirstDerivative(t);
  acc_des_minj = minJerkCurve.evaluateSecondDerivative(t);
  // check end of trajectory
  for (unsigned int i = 0; i < start_pos.size(); i++) {
    ASSERT_TRUE(std::abs(pos_des_minj[i] - end_pos[i]) <= err_tol);
  }

  t = 0.;
  while(t < duration)
  {
    pos_des_minj = minJerkCurve.evaluate(t);
    vel_des_minj = minJerkCurve.evaluateFirstDerivative(t);
    acc_des_minj = minJerkCurve.evaluateSecondDerivative(t);

    // save positions
    pos_minj_traj_x.push_back(pos_des_minj[0]);
    pos_minj_traj_y.push_back(pos_des_minj[1]);
    pos_minj_traj_z.push_back(pos_des_minj[2]);

    // save velocities
    vel_minj_traj_x.push_back(vel_des_minj[0]);
    vel_minj_traj_y.push_back(vel_des_minj[1]);
    vel_minj_traj_z.push_back(vel_des_minj[2]);

    // save accelerations
    acc_minj_traj_x.push_back(acc_des_minj[0]);
    acc_minj_traj_y.push_back(acc_des_minj[1]);
    acc_minj_traj_z.push_back(acc_des_minj[2]);

    t_traj.push_back(t);
    t += dt;
  }

  if(b_plot)
  {
    Plot2D plot_z_pos;
    plot_z_pos.xlabel("t");
    plot_z_pos.ylabel("p_z");
    plot_z_pos.drawCurve(t_traj, pos_minj_traj_z).label("min jerk spline");
    plot_z_pos.legend().atTopRight();

    Plot2D plot_z_vel;
    plot_z_vel.xlabel("t");
    plot_z_vel.ylabel("v_z");
    plot_z_vel.drawCurve(t_traj, vel_minj_traj_z).label("min jerk spline");
    plot_z_vel.legend().atTopRight();

    Plot2D plot_z_acc;
    plot_z_acc.xlabel("t");
    plot_z_acc.ylabel("a_z");
    plot_z_acc.drawCurve(t_traj, acc_minj_traj_z).label("min jerk spline");
    plot_z_acc.legend().atTopRight();

    plot_z_pos.grid();

    // Create figure to hold plot_z_pos, vel, acc (vertically)
    Figure fig = {{plot_z_pos}, {plot_z_vel}, {plot_z_acc}};
    // Create canvas to hold figure
    Canvas canvas = {{fig}};

    canvas.size(750, 750);

    // Show the plot_z_pos in a pop-up window
    canvas.show();
  }


}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}