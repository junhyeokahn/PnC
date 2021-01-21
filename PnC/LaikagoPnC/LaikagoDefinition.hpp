#pragma once
namespace Laikago {
constexpr int n_bodynode = 23;
constexpr int n_dof = 22;
constexpr int n_vdof = 6;
constexpr int n_adof = 16; //10 for draco 
constexpr int num_leg_joint = 4;
}  // namespace Laikago

namespace LaikagoBodyNode {
constexpr int basePosX = 0;
constexpr int basePosY = 1;
constexpr int basePosZ = 2;
constexpr int baseRotZ = 3;
constexpr int baseRotY = 4;
constexpr int trunk = 5;
constexpr int IMU = 6;
constexpr int FR_hip= 7;
constexpr int FR_thigh= 8;
constexpr int FR_calf= 9;
constexpr int FR_foot= 10;
constexpr int FL_hip= 11;
constexpr int FL_thigh= 12;
constexpr int FL_calf= 13;
constexpr int FL_foot= 14;
constexpr int RR_hip= 15;
constexpr int RR_thigh= 16;
constexpr int RR_calf= 17;
constexpr int RR_foot= 18;
constexpr int RL_hip= 19;
constexpr int RL_thigh= 20;
constexpr int RL_calf= 21;
constexpr int RL_foot= 22;
}  // namespace LaikagoBodyNode

namespace LaikagoDoF {
constexpr int basePosX = 0;
constexpr int basePosY = 1;
constexpr int basePosZ = 2;
constexpr int baseRotZ = 3;
constexpr int baseRotY = 4;
constexpr int baseRotX = 5;
constexpr int FR_hip_joint= 6;
constexpr int FR_thigh_joint= 7;
constexpr int FR_calf_joint= 8;
constexpr int FR_foot_fixed= 9;
constexpr int FL_hip_joint= 10;
constexpr int FL_thigh_joint= 11;
constexpr int FL_calf_joint= 12;
constexpr int FL_foot_fixed= 13;
constexpr int RR_hip_joint= 14;
constexpr int RR_thigh_joint= 15;
constexpr int RR_calf_joint= 16;
constexpr int RR_foot_fixed= 17;
constexpr int RL_hip_joint= 18;
constexpr int RL_thigh_joint= 19;
constexpr int RL_calf_joint= 20;
constexpr int RL_foot_fixed= 21;
}  // namespace LaikagoDoF

namespace LaikagoAux {
constexpr double servo_rate = 1.0 / 1000.0;
}
