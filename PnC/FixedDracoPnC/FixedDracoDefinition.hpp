#pragma once

namespace FixedDracoBodyNode {
constexpr int Torso = 0;
constexpr int lHipYaw = 1;
constexpr int lHipRoll = 2;
constexpr int lHipPitch = 3;
constexpr int lKnee = 4;
constexpr int lAnkle = 5;
constexpr int rHipYaw = 6;
constexpr int rHipRoll = 7;
constexpr int rHipPitch = 8;
constexpr int rKnee = 9;
constexpr int rAnkle = 10;
}  // namespace FixedDracoBodyNode

namespace FixedDracoDoF {
constexpr int lHipYaw = 0;
constexpr int lHipRoll = 1;
constexpr int lHipPitch = 2;
constexpr int lKnee = 3;
constexpr int lAnkle = 4;
constexpr int rHipYaw = 5;
constexpr int rHipRoll = 6;
constexpr int rHipPitch = 7;
constexpr int rKnee = 8;
constexpr int rAnkle = 9;
}  // namespace FixedDracoDoF

namespace FixedDracoAux {
constexpr double ServoRate = 1.0 / 1000.0;
}
