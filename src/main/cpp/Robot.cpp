/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>
#include <vector>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>

#include "Constructor.h"
#include "ModuleBase.h"

// All Module Includes
#include "DriveBaseModule.h"
#include "ErrorModule.h"
#include "BrownoutModule.h"

void Robot::RobotInit() {
  /**
   * There must be a fixed order for the modules for the constructor to work.
   * Your module must respect this order, or it will cause web construction failures:
   * (modules[positionInModuleVector])
   * modules[0]: Error Reporting Module
   * modules[1]: DriveBase Module
   * modules[2]: Brownout Protection Module
   * modules[3]: Autonomous Module
   * modules[4]: Pneumatics Module
   * modules[5]: Internet/Hardware Ethernet Module
   * modules[6]: Shuffleboard Module
   * modules[7]: Error Reporting File IO Thread
   * Manipulators, Sensors, etc.: TBD
   * See Error.h for relevant macros
   */

  if (!Constructor::constructThreadedRobot(std::vector<ModuleBase*> {new ErrorModule, new DriveBaseModule, new BrownoutModule}, this)) { // Pass a reference of this object to all modules
    frc::DriverStation::ReportError("[Constructor] Web Construction has failed; ensure it is acyclic and constructable");
    return;
  }
  
  frc::DriverStation::ReportError("[Constructor] Successfully Constructed Web, ModuleBase::init() has been called on all modules.");

}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {


}

void Robot::AutonomousPeriodic() {
    
}

void Robot::TeleopInit() {
  
}

void Robot::TeleopPeriodic() {
 
 
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif