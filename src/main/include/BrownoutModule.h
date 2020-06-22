#ifndef BROWNOUTMODULE_H
#define BROWNOUTMODULE_H

#include <vector>
#include <math.h> 
#include <string>
#include <fstream>


#include "Macros.h"
#include "ModuleBase.h"
#include "GenericPipe.h"

#include <frc/Timer.h>
#include <frc/PowerDistributionPanel.h>
#include <frc/DriverStation.h>

#define VOLTAGE_THRESHOLD 7.0
class BrownoutModule : public ModuleBase {
 
  GenericPipe* ErrorModulePipe;
  GenericPipe* DriveBaseModulePipe;
  frc::PowerDistributionPanel* pdp;

  void writeData(std::string fileName);
  const std::string fileName = "/";
  double getBatteryPower();
  double getMaxCurrentDraw();
  bool isBrownout();
  double getBatteryResistance();


  public:

  std::vector<uint8_t> getConstructorArgs();
  void periodicInit();
  void periodicRoutine();

  
 
};

#endif
