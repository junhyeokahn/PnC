#pragma once
namespace Scorpio {
constexpr int n_bodynode = 20;                                                                                                                                                                            
constexpr int n_dof = 11;                                                                                                                                                                            
constexpr int n_vdof = 4;                                                                             
constexpr int n_adof = 7;
}

namespace ScorpioBodyNode{
    constexpr int tower = 0;                                                                              
    constexpr int scorpio_mount = 1;                                                                      
    constexpr int scorpio_ground = 2;                                                                     
    constexpr int link1 = 3;                                                                              
    constexpr int link2 = 4;                                                                              
    constexpr int link3 = 5;                                                                              
    constexpr int link4 = 6;                                                                              
    constexpr int link4_end = 7;                                                                          
    constexpr int link5 = 8;                                                                              
    constexpr int link6 = 9;                                                                              
    constexpr int link7 = 10;                                                                             
    constexpr int link8 = 11;                                                                             
    constexpr int link8_end = 12;                                                                         
    constexpr int link7_end = 13;                                                                         
    constexpr int link9 = 14;                                                                             
    constexpr int scorpio_wrist_pitch = 15;                                                               
    constexpr int gripper_attachment_point = 16;                                                          
    constexpr int wrist_ft = 17;                                                                          
    constexpr int gripper_body = 18;                                                                      
    constexpr int end_effector = 19;      
}


namespace ScorpioDoF{
    constexpr int joint1 = 0;                                                                             
    constexpr int joint2 = 1;                                                                             
    constexpr int joint3 = 2;                                                                             
    constexpr int joint4 = 3;                                                                             
    constexpr int joint5 = 4;                                                                             
    constexpr int joint6 = 5;                                                                             
    constexpr int joint7 = 6;                                                                             
    constexpr int joint8 = 7;                                                                             
    constexpr int joint9 = 8;                                                                             
    constexpr int joint10 = 9;                                                                            
    constexpr int joint11 = 10;  
}

namespace ScorpioAux{
constexpr double ServoRate = 1.0 / 1000.0;
}
