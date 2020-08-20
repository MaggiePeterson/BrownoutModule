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
#define LMOTOR_LEAD_CHANNEL 0.0
#define RMOTOR_LEAD_CHANNEL 0.0
#define LMOTOR_FOLLOWER_CHANNEL 0.0
#define RMOTOR_FOLLOWER_CHANNEL 0.0

class BrownoutModule : public ModuleBase {
 
  GenericPipe* ErrorModulePipe;
  GenericPipe* DriveBaseModulePipe;
  frc::PowerDistributionPanel* pdp;
  double batteryResistance = 0.0;
  bool hasRun = false;
  bool fileEmpty = true;
  const std::string fileName = "BrownoutData.csv";
  const std::string energyLog = "EnergyLog.csv";
  std::ofstream myFile;
  std::ofstream energyStream;
  std::ifstream energyInStream;

  bool writeData(std::string fileName);
  double getBatteryPower();
  bool willBrownOut();
  bool isBrownout();
  double getBatteryResistance();
  void calculateBatteryResistance();
  void getLineOfBestFitSlope(double curr, double volt);
  bool accumulatePower(double time);

  double energyThisMatch = 0;
  double getMotorCurrentDraw();
  double getDriveCurrentLimitScaling();
  double nonMotorCurrent;
  double totalCurrLimit;

  double currCurrent = 0;
  double currVoltage = 0;

  std::vector<double> energyLogTime;
  std::vector<double> pastEnergyLog;

  double xsum = 0.0, x2sum = 0.0, ysum = 0.0, xysum = 0.0;

  public:

  std::vector<uint8_t> getConstructorArgs();
  void periodicInit();
  void periodicRoutine();

  /* TODO 
    Energy match data
    motor module
  */ 
 
};

#endif
