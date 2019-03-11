#pragma once

namespace AtlasBodyNode {
constexpr int pelvis = 0;
constexpr int ltorso = 1;
constexpr int mtorso = 2;
constexpr int utorso = 3;
constexpr int l_clav = 4;
constexpr int l_scap = 5;
constexpr int l_uarm = 6;
constexpr int l_larm = 7;
constexpr int l_ufarm = 8;
constexpr int l_lfarm = 9;
constexpr int l_hand = 10;
constexpr int head = 11;
constexpr int r_clav = 12;
constexpr int r_scap = 13 ;
constexpr int r_uarm = 14;
constexpr int r_larm = 15;
constexpr int r_ufarm = 16;
constexpr int r_lfarm = 17; 
constexpr int r_hand = 18;
constexpr int l_uglut = 19 ;
constexpr int l_lglut = 20;
constexpr int l_uleg = 21;
constexpr int l_lleg = 22 ;
constexpr int l_talus =23 ;
constexpr int L_foot = 24 ;
constexpr int r_uglut = 25 ;
constexpr int r_lglut = 26 ;
constexpr int r_uleg = 27 ;
constexpr int r_lleg = 28 ;
constexpr int r_talus = 29 ;
constexpr int r_foot = 30;
}  // namespace AtlasBodyNode

namespace AtlasDoF {
constexpr int back_bkz = 0;                                                                                                       
constexpr int back_bky = 1;                                                                                                              
constexpr int back_bkx = 2;                                                                                                   
constexpr int l_arm_shz = 3;                                                                                                 
constexpr int l_arm_shx = 4;                                                                                                             
constexpr int l_arm_ely = 5;                                                                                                             
constexpr int l_arm_elx = 6;                                                                                                             
constexpr int l_arm_wry = 7;                                                                                                            
constexpr int l_arm_wrx = 8;                                                                                                           
constexpr int l_arm_wry2 = 9;                                                                                                       
constexpr int neck_ry = 10;                                                                                                        
constexpr int r_arm_shz = 11;                                                                                                     
constexpr int r_arm_shx = 12;                                                                                                    
constexpr int r_arm_ely = 13;                                                                                                  
constexpr int r_arm_elx = 14;                                                                                                
constexpr int r_arm_wry = 15;                                                                                                
constexpr int r_arm_wrx = 16;                                                                                              
constexpr int r_arm_wry2 = 17;                                                                                                          
constexpr int l_leg_hpz = 18;                                                                                                           
constexpr int l_leg_hpx = 19;                                                                                                           
constexpr int l_leg_hpy = 20;                                                                                                         
constexpr int l_leg_kny = 21;                                                                                                            
constexpr int l_leg_aky = 22;                                                                                                          
constexpr int l_leg_akx = 23;                                                                                                         
constexpr int r_leg_hpz = 24;                                                                                                            
constexpr int r_leg_hpx = 25;                                                                                                          
constexpr int r_leg_hpy = 26;                                                                                                         
constexpr int r_leg_kny = 27;                                                                                                       
constexpr int r_leg_aky = 28;                                                                                                     
constexpr int r_leg_akx = 29;
}  // namespace AtlasDoF

namespace AtlasAux {
constexpr double ServoRate = 1.0 / 1000.0;
}
