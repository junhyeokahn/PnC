#pragma once
#include <PnC/Test.hpp>

class RobotSystem;
class ScorpioStateProvider;

enum GRASPING_TEST_PHASE {HOLD_PH = 0,
                          MOVE_PH = 1,
                          NUM_GRASPING_PH = 2};

class GraspingTest : public Test {
   public:
    GraspingTest(RobotSystem* robot);
    virtual ~GraspingTest();

    virtual void TestInitialization();

    void SetMovingTarget(Eigen::VectorXd pos);

   protected:
    void _ParameterSetting();
    virtual int _NextPhase(const int& phase);

    Controller* gripper_ctrl_;
    Controller* move_ctrl_;
    YAML::Node cfg_;
    ScorpioStateProvider* sp_;

};
