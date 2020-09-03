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
#define ENERGY_THRESHOLD 0.20
#define LMOTOR_LEAD_CHANNEL 0.0
#define RMOTOR_LEAD_CHANNEL 0.0
#define LMOTOR_FOLLOWER_CHANNEL 0.0
#define RMOTOR_FOLLOWER_CHANNEL 0.0

#define TELEOP_LENGTH 150
#define MATCH_LENGTH 165

class BrownoutModule : public ModuleBase {
 
  GenericPipe* ErrorModulePipe;
  GenericPipe* DriveBaseModulePipe;
  frc::PowerDistributionPanel* pdp;

  double batteryResistance = 0.0;
  bool hasRun = false;
  bool fileEmpty = true;
  double nonDriveLoad;
  double totalCurrLimit;
  std::vector<double> timeInterval;
  std::vector<double> pastEnergy;
  double xsum = 0.0, x2sum = 0.0, ysum = 0.0, xysum = 0.0; //calculating resistance

  const std::string generalFile = "/home/lvuser/BrownoutData.csv"; 
  const std::string lastMatchFile = "/home/lvuser/lastMatch.csv";
  const std::string allMatchesFile = "/home/lvuser/allMatches.csv";
  const std::string sumAllMatchesFile = "/home/lvuser/sumAllMatches.csv";
  std::fstream generalStream;
  std::fstream lastMatchStream;
  std::fstream allMatchesStream;
  std::fstream sumAllMatchesStream;
  
  void compilePastMatchData();
  bool writeData(std::string fileName);
  double getBatteryPower();
  bool willBrownOut();
  bool isBrownout();
  void calculateBatteryResistance();
  void takeSum(double curr, double volt);
  bool checkEnergy(double time, double matchTime);
  double getMotorCurrentDraw();
  double getDriveCurrentLimitScaling();

  public:

  std::vector<uint8_t> getConstructorArgs();
  void periodicInit();
  void periodicRoutine();
 
};

#endif
