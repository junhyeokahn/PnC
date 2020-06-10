#pragma once
namespace Draco {
constexpr int n_bodynode = 26;
constexpr int n_dof = 16;
constexpr int n_vdof = 6;
constexpr int n_adof = 10;
constexpr int num_leg_joint = 5;
}  // namespace Draco

namespace DracoBodyNode {
constexpr int basePosX = 0;
constexpr int basePosY = 1;
constexpr int basePosZ = 2;
constexpr int baseRotZ = 3;
constexpr int baseRotY = 4;
constexpr int Torso = 5;
constexpr int IMU = 6;
constexpr int cTorsoLed = 7;
constexpr int lHipYaw = 8;
constexpr int lHipRoll = 9;
constexpr int lHipPitch = 10;
constexpr int lKnee = 11;
constexpr int lAnkle = 12;
constexpr int lFootBack = 13;
constexpr int lFootCenter = 14;
constexpr int lFootFront = 15;
constexpr int lTorsoLed = 16;
constexpr int rHipYaw = 17;
constexpr int rHipRoll = 18;
constexpr int rHipPitch = 19;
constexpr int rKnee = 20;
constexpr int rAnkle = 21;
constexpr int rFootBack = 22;
constexpr int rFootCenter = 23;
constexpr int rFootFront = 24;
constexpr int rTorsoLed = 25;
}  // namespace DracoBodyNode

namespace DracoDoF {
constexpr int basePosX = 0;
constexpr int basePosY = 1;
constexpr int basePosZ = 2;
constexpr int baseRotZ = 3;
constexpr int baseRotY = 4;
constexpr int baseRotX = 5;
constexpr int lHipYaw = 6;
constexpr int lHipRoll = 7;
constexpr int lHipPitch = 8;
constexpr int lKnee = 9;
constexpr int lAnkle = 10;
constexpr int rHipYaw = 11;
constexpr int rHipRoll = 12;
constexpr int rHipPitch = 13;
constexpr int rKnee = 14;
constexpr int rAnkle = 15;
}  // namespace DracoDoF

namespace DracoAux {
constexpr double ServoRate = 1.0 / 1000.0;
}
