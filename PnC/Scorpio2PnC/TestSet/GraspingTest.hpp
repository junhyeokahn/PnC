#pragma once
#include <PnC/Test.hpp>

class RobotSystem;
class Scorpio2StateProvider;

enum GRASPING2_TEST_PHASE {HOLD2_PH = 0,
                          MOVE2_PH = 1,
                          NUM_GRASPING2_PH = 2};

class Grasping2Test : public Test {
   public:
    Grasping2Test(RobotSystem* robot);
    virtual ~Grasping2Test();

    virtual void TestInitialization();

    void SetMovingTarget(Eigen::VectorXd pos);

   protected:
    void _ParameterSetting();
    virtual int _NextPhase(const int& phase);

    Controller* gripper_ctrl_;
    Controller* move_ctrl_;
    YAML::Node cfg_;
    Scorpio2StateProvider* sp_;

};
