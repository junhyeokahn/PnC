#pragma once

#include <PnC/Test.hpp>

class RobotSystem;

enum BASIC_TEST_PHASE{
  BASIC = 0,
  NUM_BASIC_TEST = 1
};

class BasicTest: public Test{
public:
  BasicTest(RobotSystem* _robot);
  virtual ~BasicTest();

  virtual void TestInitialization();

protected:
  void _ParameterSetting();
  virtual int _NextPhase(const int & phase);

  Controller* basic_ctrl_;

  YAML::Node cfg_;
};
