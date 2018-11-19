#pragma once

#include <PnC/WBC/ContactSpec.hpp>

class RobotSystem;
class DracoStateProvider;

class FixedBodyContact: public ContactSpec{
public:
  FixedBodyContact(RobotSystem* _robot);
  virtual ~FixedBodyContact();

protected:
  virtual bool _UpdateContactGeometry();
  virtual bool _UpdateJc();
  virtual bool _UpdateJcDotQdot();
  virtual bool _UpdateUf();
  virtual bool _UpdateInequalityVector();
};
