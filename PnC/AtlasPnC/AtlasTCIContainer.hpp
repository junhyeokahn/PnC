#pragma once

#include <PnC/WBC/BasicContact.hpp>
#include <PnC/WBC/BasicTask.hpp>
#include <PnC/WBC/TCIContainer.hpp>

class AtlasTCIContainer : public TCIContainer {
public:
  AtlasTCIContainer(RobotSystem *_robot);
  virtual ~AtlasTCIContainer();

  Task *com_task;
  Task *pelvis_ori_task;
  Task *upper_body_task;
  Task *rfoot_pos_task;
  Task *rfoot_ori_task;
  Task *lfoot_pos_task;
  Task *lfoot_ori_task;

  Contact *rfoot_contact;
  Contact *lfoot_contact;

protected:
  /* data */
};
