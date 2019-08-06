#pragma once
namespace Valkyrie {
constexpr int n_bodynode = 81;
constexpr int n_dof = 34;
constexpr int n_vdof = 6;
constexpr int n_adof = 28;

}  // namespace Valkyrie

namespace ValkyrieBodyNode {
constexpr int basePosX = 0;
constexpr int basePosY = 1;
constexpr int basePosZ = 2;
constexpr int baseRotZ = 3;
constexpr int baseRotY = 4;
constexpr int pelvis = 5;
constexpr int leftHipYawLink = 6;
constexpr int leftHipRollLink = 7;
constexpr int leftHipPitchLink = 8;
constexpr int leftKneePitchLink = 9;
constexpr int leftAnklePitchLink = 10;
constexpr int leftFoot = 11;
constexpr int leftCOP_Frame = 12;
constexpr int pelvisMiddleImu_Frame = 13;
constexpr int pelvisRearImu_Frame = 14;
constexpr int rightHipYawLink = 15;
constexpr int rightHipRollLink = 16;
constexpr int rightHipPitchLink = 17;
constexpr int rightKneePitchLink = 18;
constexpr int rightAnklePitchLink = 19;
constexpr int rightFoot = 20;
constexpr int rightCOP_Frame = 21;
constexpr int torsoYawLink = 22;
constexpr int torsoPitchLink = 23;
constexpr int torso = 24;
constexpr int leftHazardCamera_Frame = 25;
constexpr int leftShoulderPitchLink = 26;
constexpr int leftShoulderRollLink = 27;
constexpr int leftShoulderYawLink = 28;
constexpr int leftElbowPitchLink = 29;
constexpr int leftForearmLink = 30;
constexpr int leftWristRollLink = 31;
constexpr int leftPalm = 32;
constexpr int leftIndexFingerPitch1Link = 33;
constexpr int leftIndexFingerPitch2Link = 34;
constexpr int leftIndexFingerPitch3Link = 35;
constexpr int leftMiddleFingerPitch1Link = 36;
constexpr int leftMiddleFingerPitch2Link = 37;
constexpr int leftMiddleFingerPitch3Link = 38;
constexpr int leftPinkyPitch1Link = 39;
constexpr int leftPinkyPitch2Link = 40;
constexpr int leftPinkyPitch3Link = 41;
constexpr int leftThumbRollLink = 42;
constexpr int leftThumbPitch1Link = 43;
constexpr int leftThumbPitch2Link = 44;
constexpr int leftThumbPitch3Link = 45;
constexpr int leftTorsoImu_Frame = 46;
constexpr int lowerNeckPitchLink = 47;
constexpr int neckYawLink = 48;
constexpr int upperNeckPitchLink = 49;
constexpr int head = 50;
constexpr int center_bottom_led_frame = 51;
constexpr int center_top_led_frame = 52;
constexpr int head_imu_link = 53;
constexpr int hokuyo_link = 54;
constexpr int head_hokuyo_frame = 55;
constexpr int left_camera_frame = 56;
constexpr int left_led_frame = 57;
constexpr int right_camera_frame = 58;
constexpr int right_led_frame = 59;
constexpr int rightHazardCamera_Frame = 60;
constexpr int rightShoulderPitchLink = 61;
constexpr int rightShoulderRollLink = 62;
constexpr int rightShoulderYawLink = 63;
constexpr int rightElbowPitchLink = 64;
constexpr int rightForearmLink = 65;
constexpr int rightWristRollLink = 66;
constexpr int rightPalm = 67;
constexpr int rightIndexFingerPitch1Link = 68;
constexpr int rightIndexFingerPitch2Link = 69;
constexpr int rightIndexFingerPitch3Link = 70;
constexpr int rightMiddleFingerPitch1Link = 71;
constexpr int rightMiddleFingerPitch2Link = 72;
constexpr int rightMiddleFingerPitch3Link = 73;
constexpr int rightPinkyPitch1Link = 74;
constexpr int rightPinkyPitch2Link = 75;
constexpr int rightPinkyPitch3Link = 76;
constexpr int rightThumbRollLink = 77;
constexpr int rightThumbPitch1Link = 78;
constexpr int rightThumbPitch2Link = 79;
constexpr int rightThumbPitch3Link = 80;
}  // namespace ValkyrieBodyNode

namespace ValkyrieDoF {
constexpr int basePosX = 0;
constexpr int basePosY = 1;
constexpr int basePosZ = 2;
constexpr int baseRotZ = 3;
constexpr int baseRotY = 4;
constexpr int pelvis = 5;
constexpr int leftHipYaw = 6;
constexpr int leftHipRoll = 7;
constexpr int leftHipPitch = 8;
constexpr int leftKneePitch = 9;
constexpr int leftAnklePitch = 10;
constexpr int leftAnkleRoll = 11;
constexpr int rightHipYaw = 12;
constexpr int rightHipRoll = 13;
constexpr int rightHipPitch = 14;
constexpr int rightKneePitch = 15;
constexpr int rightAnklePitch = 16;
constexpr int rightAnkleRoll = 17;
constexpr int torsoYaw = 18;
constexpr int torsoPitch = 19;
constexpr int torsoRoll = 20;
constexpr int leftShoulderPitch = 21;
constexpr int leftShoulderRoll = 22;
constexpr int leftShoulderYaw = 23;
constexpr int leftElbowPitch = 24;
constexpr int leftForearmYaw = 25;
constexpr int lowerNeckPitch = 26;
constexpr int neckYaw = 27;
constexpr int upperNeckPitch = 28;
constexpr int rightShoulderPitch = 29;
constexpr int rightShoulderRoll = 30;
constexpr int rightShoulderYaw = 31;
constexpr int rightElbowPitch = 32;
constexpr int rightForearmYaw = 33;
}  // namespace ValkyrieDoF

namespace ValkyrieAux {
constexpr double servo_rate = 0.001;
}
