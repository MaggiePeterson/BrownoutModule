#ifndef BROWNOUTMODULE_H
#define BROWNOUTMODULE_H

#include <vector>
#include <math.h> 

#include "Macros.h"
#include "ModuleBase.h"
#include "GenericPipe.h"


class BrownoutModule : public ModuleBase {
 
  GenericPipe* ErrorModulePipe;
  GenericPipe* BrownoutModulePipe;
  GenericPipe* AutonomousModulePipe;

  public:

  std::vector<uint8_t> getConstructorArgs();
  void periodicInit();
  void periodicRoutine();
 
};

#endif
