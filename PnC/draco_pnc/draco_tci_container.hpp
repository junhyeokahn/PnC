#pragma once

#include "PnC/WBC/BasicContact.hpp"
#include "PnC/WBC/BasicTask.hpp"
#include "PnC/WBC/TCIContainer.hpp"
#include "PnC/draco_pnc/draco_rolling_joint_constraint.hpp"

class DracoTCIContainer : public TCIContainer {
public:
  DracoTCIContainer(RobotSystem *_robot);
  virtual ~DracoTCIContainer();

  Task *joint_task;
  Task *com_task;
  Task *torso_ori_task;
  Task *upper_body_task;
  Task *rfoot_pos_task;
  Task *rfoot_ori_task;
  Task *lfoot_pos_task;
  Task *lfoot_ori_task;

  Contact *rfoot_contact;
  Contact *lfoot_contact;

  InternalConstraint *rolling_joint_constraint;

protected:
  /* data */
};
