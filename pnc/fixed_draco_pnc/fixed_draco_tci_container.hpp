#pragma once

#include "pnc/fixed_draco_pnc/fixed_draco_rolling_joint_constraint.hpp"
#include "pnc/whole_body_controllers/basic_contacts.hpp"
#include "pnc/whole_body_controllers/basic_tasks.hpp"
#include "pnc/whole_body_controllers/tci_container.hpp"

class FixedDracoTCIContainer : public TCIContainer {
public:
  FixedDracoTCIContainer(RobotSystem *_robot);
  virtual ~FixedDracoTCIContainer();

  Task *joint_task;
  // Task *upper_body_task;
  Task *neck_task;
  Task *rfoot_pos_task;
  Task *rfoot_ori_task;
  Task *lfoot_pos_task;
  Task *lfoot_ori_task;
  Task *rhand_pos_task;
  Task *rhand_ori_task;
  Task *lhand_pos_task;
  Task *lhand_ori_task;

  InternalConstraint *rolling_joint_constraint;

protected:
  /* data */
};
