#pragma once

#include "pnc/draco_pnc/draco_rolling_joint_constraint.hpp"
#include "pnc/whole_body_controllers/basic_contacts.hpp"
#include "pnc/whole_body_controllers/basic_tasks.hpp"
#include "pnc/whole_body_controllers/tci_container.hpp"

class DracoTCIContainer : public TCIContainer {
public:
  DracoTCIContainer(RobotSystem *_robot);
  virtual ~DracoTCIContainer();

  Task *cam_task;
  Task *joint_task;
  Task *com_task;
  Task *torso_ori_task;
  Task *upper_body_task;
  Task *rfoot_pos_task;
  Task *rfoot_ori_task;
  Task *lfoot_pos_task;
  Task *lfoot_ori_task;
  Task *lhand_pos_task;
  Task *lhand_ori_task;
  Task *rhand_pos_task;
  Task *rhand_ori_task;

  Contact *rfoot_contact;
  Contact *lfoot_contact;

  InternalConstraint *rolling_joint_constraint;

protected:
  /* data */
};
