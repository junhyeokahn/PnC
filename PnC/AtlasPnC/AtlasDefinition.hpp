#pragma once
namespace Atlas {
constexpr int n_bodynode = 38;
constexpr int n_dof = 29;
constexpr int n_vdof = 6;
constexpr int n_adof = 23;
constexpr int num_leg_joint = 6;
}  // namespace Atlas

namespace AtlasBodyNode {
constexpr int basePosX = 0;
constexpr int basePosY = 1;
constexpr int basePosZ = 2;
constexpr int baseRotZ = 3;
constexpr int baseRotY = 4;
constexpr int pelvis = 5;
constexpr int ltorso = 6;
constexpr int mtorso = 7;
constexpr int utorso = 8;
constexpr int l_clav = 9;
constexpr int l_scap = 10;
constexpr int l_uarm = 11;
constexpr int l_larm = 12;
constexpr int l_ufarm = 13;
constexpr int l_lfarm = 14;
constexpr int l_hand = 15;
constexpr int head = 16;
constexpr int r_clav = 17;
constexpr int r_scap = 18;
constexpr int r_uarm = 19;
constexpr int r_larm = 20;
constexpr int r_ufarm = 21;
constexpr int r_lfarm = 22;
constexpr int r_hand = 23;
constexpr int l_uglut = 24;
constexpr int l_lglut = 25;
constexpr int l_uleg = 26;
constexpr int l_lleg = 27;
constexpr int l_talus = 28;
constexpr int l_foot = 29;
constexpr int l_sole = 30;
constexpr int r_uglut = 31;
constexpr int r_lglut = 32;
constexpr int r_uleg = 33;
constexpr int r_lleg = 34;
constexpr int r_talus = 35;
constexpr int r_foot = 36;
constexpr int r_sole = 37;
}  // namespace AtlasBodyNode

namespace AtlasDoF {
constexpr int basePosX = 0;
constexpr int basePosY = 1;
constexpr int basePosZ = 2;
constexpr int baseRotZ = 3;
constexpr int baseRotY = 4;
constexpr int pelvis = 5;
constexpr int back_bkz = 6;
constexpr int back_bky = 7;
constexpr int back_bkx = 8;
constexpr int l_arm_shz = 9;
constexpr int l_arm_shx = 10;
constexpr int l_arm_ely = 11;
constexpr int l_arm_elx = 12;
constexpr int r_arm_shz = 13;
constexpr int r_arm_shx = 14;
constexpr int r_arm_ely = 15;
constexpr int r_arm_elx = 16;
constexpr int l_leg_hpz = 17;
constexpr int l_leg_hpx = 18;
constexpr int l_leg_hpy = 19;
constexpr int l_leg_kny = 20;
constexpr int l_leg_aky = 21;
constexpr int l_leg_akx = 22;
constexpr int r_leg_hpz = 23;
constexpr int r_leg_hpx = 24;
constexpr int r_leg_hpy = 25;
constexpr int r_leg_kny = 26;
constexpr int r_leg_aky = 27;
constexpr int r_leg_akx = 28;
}  // namespace AtlasDoF

namespace AtlasAux {
constexpr double servo_rate = 0.001;
}
