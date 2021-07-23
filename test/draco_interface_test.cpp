#include "pnc/draco_pnc/draco_interface.hpp"
#include <cstdlib>
#include <iostream>
#include <unistd.h>

int main(int argc, char *argv[]) {
  DracoInterface *interface;
  double sleep_time(2.);
  interface = new DracoInterface(true);
  delete interface;
  std::cout << "deleted" << std::endl;
  sleep(sleep_time);
  interface = new DracoInterface(true);
  delete interface;
  std::cout << "deleted" << std::endl;
  sleep(sleep_time);
  interface = new DracoInterface(false);
  delete interface;
  std::cout << "deleted" << std::endl;
  sleep(sleep_time);
  interface = new DracoInterface(true);
  delete interface;
  std::cout << "deleted" << std::endl;
  sleep(sleep_time);
  std::cout << "done" << std::endl;
  return 0;
}
