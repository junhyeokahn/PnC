#pragma once

#include <Eigen/Dense>

#include <pnc/whole_body_controllers/task.hpp>

class JointTask : public Task {
public:
  JointTask(RobotSystem *_robot);
  virtual ~JointTask(){};

private:
  void update_cmd();
  void update_jacobian();
};

class SelectedJointTask : public Task {
public:
  SelectedJointTask(RobotSystem *_robot, std::vector<std::string> _target_ids);
  virtual ~SelectedJointTask(){};

private:
  /* data */
  void update_cmd();
  void update_jacobian();
};

class LinkPosTask : public Task {
public:
  LinkPosTask(RobotSystem *_robot, std::vector<std::string> _target_ids,
              std::string _topic_name = {});
  virtual ~LinkPosTask(){};

private:
  /* data */
  void update_cmd();
  void update_jacobian();
};

class LinkOriTask : public Task {
public:
  LinkOriTask(RobotSystem *_robot, std::vector<std::string> _target_ids,
              std::string _topic_name = {});
  virtual ~LinkOriTask(){};

private:
  /* data */
  void update_cmd();
  void update_jacobian();
};

class CenterOfMassTask : public Task {
public:
  CenterOfMassTask(RobotSystem *_robot);
  virtual ~CenterOfMassTask(){};

private:
  /* data */
  void update_cmd();
  void update_jacobian();
};
