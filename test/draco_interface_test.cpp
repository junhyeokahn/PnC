//#include "pnc/draco_pnc/draco_interface.hpp"
#include "pnc/draco_pnc/draco_interface.hpp"
#include <cstdlib>
#include <iostream>
#include <unistd.h>

int main(int argc, char *argv[]) {
  DracoInterface *interface;
  DracoSensorData *sensor;
  DracoCommand *cmd;
  double sleep_time(2.);
  for (int i = 0; i < 100; ++i) {
    std::cout << "i : " << i << std::endl;
    interface = new DracoInterface();
    // sensor = new DracoSensorData();
    // cmd = new DracoCommand();
    // cmd->joint_positions["hi"] = 1.0;
    // cmd->joint_positions["hroi"] = 1.0;
    // interface->interrupt->b_interrupt_button_x = true;
    delete interface;
    // delete sensor;
    // delete cmd;
    std::cout << "destructed" << std::endl;
    // sleep(sleep_time);
  }

  std::cout << "done" << std::endl;

  return 0;
}
