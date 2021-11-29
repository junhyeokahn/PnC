#pragma once

#include <Eigen/Dense>

#include "pnc/whole_body_controllers/task.hpp"

/// class JointTask
class JointTask : public Task {
public:
  /// \{ \name Constructor and Destructor
  JointTask(RobotSystem *_robot);

  virtual ~JointTask(){};
  /// \}

private:
  void update_cmd();
  void update_jacobian();
};

/// class SelectedJointTask
class SelectedJointTask : public Task {
public:
  /// \{ \name Constructor and Destructor
  SelectedJointTask(RobotSystem *_robot, std::vector<std::string> _target_ids);

  virtual ~SelectedJointTask(){};
  /// \}

private:
  void update_cmd();
  void update_jacobian();
};

/// class LinkPosTask
class LinkPosTask : public Task {
public:
  /// \{ \name Constructor and Destructor
  LinkPosTask(RobotSystem *_robot, std::vector<std::string> _target_ids);

  virtual ~LinkPosTask(){};
  /// \}

private:
  void update_cmd();
  void update_jacobian();
};

/// class LinkOriTask
class LinkOriTask : public Task {
public:
  /// \{ \name Constructor and Destructor
  LinkOriTask(RobotSystem *_robot, std::vector<std::string> _target_ids);

  virtual ~LinkOriTask(){};
  /// \}

private:
  void update_cmd();
  void update_jacobian();
};

/// class CenterOfMassTask
class CenterOfMassTask : public Task {
public:
  /// \{ \name Constructor and Destructor
  CenterOfMassTask(RobotSystem *_robot);

  virtual ~CenterOfMassTask(){};
  /// \}

private:
  void update_cmd();
  void update_jacobian();
};
