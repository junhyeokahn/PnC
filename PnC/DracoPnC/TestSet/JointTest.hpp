#pragma once

#include <PnC/Test.hpp>

class RobotSystem;

enum JPOS_TEST_PHASE{
  JPOS_TEST_INI = 0,
  JPOS_TEST_SWING = 1,
  NUM_JPOS_TEST = 2
};

class JointTest: public Test{
public:
  JointTest(RobotSystem* _robot);
  virtual ~JointTest();

  virtual void TestInitialization();

protected:
  void _ParameterSetting();
  virtual int _NextPhase(const int & phase);

  Controller* jpos_ctrl_ini_;
  Controller* jpos_ctrl_;
};
