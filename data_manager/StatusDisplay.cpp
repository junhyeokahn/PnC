#include <stdio.h>
#include <iostream>
#include "DataSave.h"

int main (int argc, char ** argv){
  DataSave* data_save;
  if(argc>1 && (argv[1][1])=='v'){
    data_save = new DataSave(true);
  }else {
    data_save = new DataSave(false);
  }

  data_save->start();

  int dummy;
  std::cin >> dummy;

  delete data_save;
  return 0;
}
