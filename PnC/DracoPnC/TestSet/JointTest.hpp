#pragma once

#include <PnC/Test.hpp>

class RobotSystem;

enum JPOS_TEST_PHASE{
  JPOS_TEST_SWING = 0,
  NUM_JPOS_TEST = 1
};

class JointTest: public Test{
public:
  JointTest(RobotSystem* _robot);
  virtual ~JointTest();

  virtual void TestInitialization();

protected:
  void _ParameterSetting();
  virtual int _NextPhase(const int & phase);

  Controller* jpos_swing_ctrl_;

  YAML::Node cfg_;
};
