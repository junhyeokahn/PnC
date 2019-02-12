#pragma once

namespace DracoBodyNode
{
    constexpr int basePosX = 0;
    constexpr int basePosY = 1;
    constexpr int basePosZ = 2;
    constexpr int baseRotZ = 3;
    constexpr int baseRotY = 4;
    constexpr int Torso = 5;
    constexpr int cTorsoLed = 6;
    constexpr int lHipYaw = 7;
    constexpr int lHipRoll = 8;
    constexpr int lHipPitch = 9;
    constexpr int lKnee = 10;
    constexpr int lAnkle = 11;
    constexpr int lFootBack = 12;
    constexpr int lFootCenter = 13;
    constexpr int lFootFront = 14;
    constexpr int lTorsoLed = 15;
    constexpr int rHipYaw = 16;
    constexpr int rHipRoll = 17;
    constexpr int rHipPitch = 18;
    constexpr int rKnee = 19;
    constexpr int rAnkle = 20;
    constexpr int rFootBack = 21;
    constexpr int rFootCenter = 22;
    constexpr int rFootFront = 23;
    constexpr int rTorsoLed = 24;
} /* DracoBodyNode */

namespace DracoDoF
{
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
} /* DracoDoF */

namespace DracoAux
{
    constexpr double ServoRate = 1.0/1000.0;
}
