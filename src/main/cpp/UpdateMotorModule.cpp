#include "UpdateMotorModule.h"



void UpdateMotorModule::periodicInit() {
  
   this->msInterval = BrownoutModuleRunInterval;
   this->ErrorModulePipe = pipes[0];
   this->DriveBaseModulePipe = pipes[1];
   this->BrownoutModulePipe = pipes[2];
}

void UpdateMotorModule::periodicRoutine() {

     Message* msg = 0;
     while( BrownoutModulePipe->size() >0 )
            msg = BrownoutModulePipe->popQueue();
		if (!msg) return;
    
    if(msg->str == "SETPOINT")
        setMotorSetpoint(msg->val);

    else if (msg->str == "CURRENT")
        setCurrentLimit(msg->val);
  
}

//sends data to DriveBaseModule
//or does this have to update the motor itself
void UpdateMotorModule::setMotorSetpoint(double setpoint){

    DriveBaseModulePipe->pushQueue(new Message("SETPOINT", setpoint));

}

void UpdateMotorModule::setCurrentLimit(double scaling){
    
    DriveBaseModulePipe->pushQueue(new Message("CURRENT", scaling));
}

std::vector<uint8_t> UpdateMotorModule::getConstructorArgs() { return std::vector<uint8_t> {ErrorModuleID, BrownoutModuleID}; }
