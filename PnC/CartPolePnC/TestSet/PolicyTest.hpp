#pragma once

#include <PnC/Test.hpp>

class RobotSystem;

enum POLICY_TEST_PHASE{
  POLICY = 0,
  NUM_POLICY_TEST = 1
};

class PolicyTest: public Test{
public:
  PolicyTest(RobotSystem* _robot);
  virtual ~PolicyTest();

  virtual void TestInitialization();

protected:
  void _ParameterSetting();
  virtual int _NextPhase(const int & phase);

  Controller* policy_ctrl_;

  YAML::Node cfg_;
};
