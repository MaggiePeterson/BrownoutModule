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
#include <frc/RobotController.h>

#define VOLTAGE_THRESHOLD 7.0
#define LMOTOR_FRONT_CHANNEL 0.0
#define RMOTOR_FRONT_CHANNEL 0.0
#define LMOTOR_FOLLOWER_CHANNEL 0.0
#define RMOTOR_FOLLOWER_CHANNEL 0.0

class BrownoutModule : public ModuleBase {
 
  GenericPipe* ErrorModulePipe;
  GenericPipe* DriveBaseModulePipe;
  frc::PowerDistributionPanel* pdp;
  double batteryResistance = 0.0;
  bool hasRun = false;

  bool writeData(std::string fileName);
  const std::string fileName = "/";
  double getBatteryPower();
  double getMaxCurrentDraw();
  bool isBrownout();
  double getBatteryResistance();
  void calculateBatteryResistance();
  double getLineOfBestFitSlope(std::vector<double> xData, std::vector<double>yData);
  void accumulatePower(double deltaTime);
  double energyThisMatch = 0;
  double getMotorCurrentDraw();
  double getCurrentLimitScaling();


  public:

  std::vector<uint8_t> getConstructorArgs();
  void periodicInit();
  void periodicRoutine();

  
 
};

#endif
