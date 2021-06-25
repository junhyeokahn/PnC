#pragma once

#include <PnC/AtlasPnC/AtlasInterface.hpp>
#include <PnC/AtlasPnC/AtlasStateProvider.hpp>
#include <PnC/AtlasPnC/AtlasTCIContainer.hpp>
#include <PnC/RobotSystem/RobotSystem.hpp>
#include <PnC/WBC/IHWBC/IHWBC.hpp>
#include <PnC/WBC/IHWBC/JointIntegrator.hpp>

class AtlasController {
public:
  AtlasController(AtlasTCIContainer *_tci_container, RobotSystem *_robot);
  virtual ~AtlasController();

  void getCommand(void *_cmd);

private:
  /* data */
  RobotSystem *robot_;
  IHWBC *wbc_;
  JointIntegrator *joint_integrator_;

  AtlasTCIContainer *tci_container_;
  AtlasStateProvider *sp_;

  void FirstVisit();
};
