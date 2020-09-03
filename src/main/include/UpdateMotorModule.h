#ifndef UPDATEMOTORMODULE_H
#define UPDATEMOTORMODULE_H

#include <vector>
#include <math.h> 

#include "Macros.h"
#include "ModuleBase.h"
#include "GenericPipe.h"

#include "rev/CANSparkMax.h"

#define lMotorLeaderID 13 // Change these according to hardware
#define lMotorFollowerID 12 
#define rMotorLeaderID 3
#define rMotorFollowerID 50

#define motorInitMaxCurrent 60 // The initial max current setting
#define motorInitRatedCurrent 40 // The inital rated current settings
#define motorInitLimitCycles 50  // The inital number of allowed ms at peak current

#define lInvert true // Inversion setings for sides
#define rInvert false 

#define xDeadband 0.1
#define yDeadband 0.1

class UpdateMotorModule : public ModuleBase {

  GenericPipe* ErrorModulePipe;
  GenericPipe* BrownoutModulePipe;
  GenericPipe* AutonomousModulePipe;
  GenericPipe* DriveBaseModulePipe;

  public:

  std::vector<uint8_t> getConstructorArgs();
  void periodicInit();
  void periodicRoutine();
  void setMotorSetpoint(double setpoint);
  void setCurrentLimit(double scaling);

};

#endif
