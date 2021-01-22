#pragma once
namespace A1 {
constexpr int n_bodynode = 27;
constexpr int n_dof = 18;
constexpr int n_vdof = 6;
constexpr int n_adof = 12; 
constexpr int num_leg_joint = 3;
}  // namespace Valkyrie

namespace A1BodyNode {
constexpr int basePosX = 0;
constexpr int basePosY = 1;
constexpr int basePosZ = 2;
constexpr int baseRotZ = 3;
constexpr int baseRotY = 4;
constexpr int trunk = 5;
constexpr int imu_link = 6;
constexpr int FR_hip = 7;
constexpr int FR_thigh_shoulder = 8;
constexpr int FR_thigh = 9;
constexpr int FR_calf = 10;
constexpr int FR_foot = 11;
constexpr int FL_hip = 12;
constexpr int FL_thigh_shoulder = 13;
constexpr int FL_thigh = 14;
constexpr int FL_calf = 15;
constexpr int FL_foot = 16;
constexpr int RR_hip = 17;
constexpr int RR_thigh_shoulder = 18;
constexpr int RR_thigh = 19;
constexpr int RR_calf = 20;
constexpr int RR_foot = 21;
constexpr int RL_hip = 22;
constexpr int RL_thigh_shoulder = 23;
constexpr int RL_thigh = 24;
constexpr int RL_calf = 25;
constexpr int RL_foot = 26;
}  // namespace A1BodyNode

namespace A1DoF {
constexpr int basePosX = 0;
constexpr int basePosY = 1;
constexpr int basePosZ = 2;
constexpr int baseRotZ = 3;
constexpr int baseRotY = 4;
constexpr int trunk = 5;
constexpr int FR_hip_joint = 6;
constexpr int FR_thigh_joint = 7;
constexpr int FR_calf_joint = 8;
constexpr int FL_hip_joint = 9;
constexpr int FL_thigh_joint = 10;
constexpr int FL_calf_joint = 11;
constexpr int RR_hip_joint = 12;
constexpr int RR_thigh_joint = 13;
constexpr int RR_calf_joint = 14;
constexpr int RL_hip_joint = 15;
constexpr int RL_thigh_joint = 16;
constexpr int RL_calf_joint = 17;
}  // namespace A1DoF

namespace A1Aux {
constexpr double servo_rate = 2.0 / 1000.0; //A1 runs at 500 Hz
}
