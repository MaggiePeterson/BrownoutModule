#ifndef BROWNOUTMODULE_H
#define BROWNOUTMODULE_H

#include <vector>
#include <math.h> 

#include "Macros.h"
#include "ModuleBase.h"
#include "GenericPipe.h"

#include <frc/PowerDistributionPanel.h>


class BrownoutModule : public ModuleBase {
 
  GenericPipe* ErrorModulePipe;
  GenericPipe* DriveBaseModulePipe;
  frc::PowerDistributionPanel* pdp;


  public:

  std::vector<uint8_t> getConstructorArgs();
  void periodicInit();
  void periodicRoutine();
 
};

#endif
